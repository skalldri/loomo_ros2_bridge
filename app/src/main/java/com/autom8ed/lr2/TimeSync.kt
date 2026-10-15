package com.autom8ed.lr2

import android.net.Network
import android.os.SystemClock
import android.util.Log
import java.io.IOException
import java.net.DatagramPacket
import java.net.DatagramSocket
import java.net.InetAddress
import java.net.SocketTimeoutException
import java.nio.ByteBuffer
import java.util.Random
import java.util.concurrent.atomic.AtomicLong
import kotlin.concurrent.thread

/**
 * Puts the Loomo's header stamps on the Jetson's clock.
 *
 * Every stamp the app publishes comes from the Segway services (frame platformTimeStamp,
 * SensorData/Pose2D/AlgoTfData timestamps), in microseconds of the Loomo's wall clock
 * (CLOCK_REALTIME). The Loomo has no internet access and that clock ran 652.4 s ahead of the
 * Jetson's (2026-10-13), so nothing on the Jetson could combine the Loomo's TF, odometry or images
 * with its own data (the L2 LIDAR, robot_state_publisher, SLAM). ROS convention: a stamp is the
 * acquisition time on the consumer's clock.
 *
 * The app does not set the Loomo's clock (it cannot without root, and stepping it under the
 * running vision service is asking for trouble: the service drops frames whose timestamp does not
 * increase). Instead it measures the offset to the Jetson with SNTP (RFC 4330) against the
 * responder on the Jetson's end of the Loomo link (loomo_jetson_config `loomo-sntp` service) and
 * adds it to every stamp: [toJetson].
 *
 * Measurement: one query per second (a quick burst at start). Each sample gives the offset of the
 * Jetson's clock to SystemClock.elapsedRealtimeNanos() and the round-trip time; the estimate is the
 * sample with the smallest round trip among the last [WINDOW] (its error is at most half its round
 * trip, plus the drift since it was taken). The mapping from elapsed time to the Loomo's wall clock
 * is re-measured at every poll to within 50 us (an edge of System.currentTimeMillis()), so a
 * step of the Loomo's wall clock is followed within a second. A step of the Jetson's clock shows
 * as [STEP_CONFIRM] consecutive samples that disagree with the estimate, and resets the window
 * (OffsetFilter, unit tested).
 *
 * Until the first answer [isReady] is false and the publishers drop stamped messages instead of
 * publishing Loomo-clock stamps: a TF listener that once received a stamp 650 s in the future
 * rejects every correct one afterwards as too old (TF_OLD_DATA) until its cache catches up.
 *
 * Observability: one line per minute under the tag "TimeSync" (offset, round trips, drift, how the
 * Jetson reports its own synchronisation, and the age of each stamp source when converted).
 */
object TimeSync {
    const val TAG = "TimeSync"

    private const val NTP_PORT = 123
    private const val NS_PER_S = 1_000_000_000L
    private const val TIMEOUT_MS = 500
    private const val BURST = 8
    private const val BURST_INTERVAL_MS = 100L
    private const val INTERVAL_MS = 1000L
    private const val WINDOW = 16
    private const val STEP_MARGIN_NS = 5_000_000L
    private const val STEP_CONFIRM = 3
    private const val WALL_STEP_NS = 1_000_000L
    private const val EDGE_ATTEMPTS = 5
    private const val EDGE_MAX_BRACKET_NS = 100_000L
    private const val REPORT_PERIOD_MS = 60_000L
    private const val STALE_LOG_MS = 30_000L

    /** Jetson time minus Loomo wall-clock time (ns), as used by [toJetson]. */
    @Volatile var offsetNs: Long = 0
        private set

    @Volatile private var ready = false

    /** Stamped messages dropped because no offset had been measured yet. */
    private val unsyncedDrops = AtomicLong()

    /** How old each kind of SDK timestamp is when it is converted (Loomo wall clock now minus stamp). */
    class StampAge(val name: String) {
        private var n = 0L
        private var minUs = Long.MAX_VALUE
        private var maxUs = Long.MIN_VALUE

        @Synchronized fun add(ageUs: Long) {
            n++
            if (ageUs < minUs) minUs = ageUs
            if (ageUs > maxUs) maxUs = ageUs
        }

        @Synchronized fun takeReport(): String? {
            if (n == 0L) return null
            val s = String.format("%s n=%d %.1f..%.1f ms", name, n, minUs / 1e3, maxUs / 1e3)
            n = 0; minUs = Long.MAX_VALUE; maxUs = Long.MIN_VALUE
            return s
        }
    }

    private val AGES = java.util.concurrent.CopyOnWriteArrayList<StampAge>()

    /** A stamp source to report in the minute line, e.g. one per camera stream. */
    fun stampAge(name: String): StampAge = StampAge(name).also { AGES.add(it) }

    val POSE_2D = stampAge("pose2d")
    val BASE_IMU = stampAge("base_imu")
    val TF_ALGO = stampAge("tf")

    /** True once stamps can be converted; publishers drop stamped messages before that. */
    fun isReady(): Boolean = ready

    /** Counts a stamped message dropped because [isReady] was false. */
    fun dropUnsynced() {
        val n = unsyncedDrops.incrementAndGet()
        if (n == 1L) Log.w(TAG, "dropping stamped messages until the first clock sample arrives")
    }

    /** Sets [stamp] to the Jetson time of an event at [loomoUs] (Loomo wall clock, microseconds). */
    fun toJetson(stamp: builtin_interfaces.msg.Time, loomoUs: Long, age: StampAge? = null) {
        if (age != null) age.add(System.currentTimeMillis() * 1000 - loomoUs)
        val ns = loomoUs * 1000 + offsetNs
        stamp.sec = (ns / NS_PER_S).toInt()
        stamp.nanosec = (ns % NS_PER_S).toInt()
    }

    /**
     * [loomoUs] (Loomo wall clock, microseconds) on the Jetson's clock, in microseconds. For a value
     * that is formatted later or more than once and must stay identical (a camera frame's stamp).
     */
    fun toJetsonUs(loomoUs: Long, age: StampAge? = null): Long {
        if (age != null) age.add(System.currentTimeMillis() * 1000 - loomoUs)
        return loomoUs + offsetNs / 1000
    }

    /** The Loomo's wall clock now, in microseconds (millisecond resolution), for [toJetson]. */
    fun nowLoomoUs(): Long = System.currentTimeMillis() * 1000

    /**
     * Starts measuring against [server] (normally the Jetson, 10.0.0.1) over [network], or, with
     * [enabled] false, publishes Loomo-clock stamps as before (offset 0).
     */
    fun start(network: Network?, server: String, enabled: Boolean) {
        if (!enabled) {
            Log.w(TAG, "clock sync disabled by configuration: stamps stay on the Loomo's clock")
            ready = true
            return
        }
        Log.i(TAG, "measuring the clock offset to $server:$NTP_PORT")
        thread(name = "time-sync", isDaemon = true) {
            try {
                Sync(network, server).run()
            } catch (t: Throwable) {
                Log.e(TAG, "clock sync thread died; keeping offset ${offsetNs / 1e6} ms", t)
            }
        }
    }

    private class Sync(private val network: Network?, private val server: String) {
        // Offsets to elapsed realtime (immune to steps of the Loomo's wall clock).
        private val filter = OffsetFilter(WINDOW, STEP_MARGIN_NS, STEP_CONFIRM)
        private val startedAtElapsedNs = SystemClock.elapsedRealtimeNanos()
        private val nonce = Random()
        private val request = ByteArray(48)
        private val response = ByteArray(128)
        private var wallMinusElapsedNs = 0L
        private var polls = 0L
        private var answers = 0L
        private var timeouts = 0L
        private var errors = 0L
        private var steps = 0L
        private var edgeFailures = 0L
        private var lastAnswerElapsedNs = 0L
        private var lastStaleLogElapsedNs = 0L
        private var jetsonLeap = -1
        private var reportAtElapsedNs = 0L
        private var reportOffsetToElapsedNs = 0L
        private val reportRtts = ArrayList<Long>()

        fun run() {
            val address = InetAddress.getByName(server)
            val socket = DatagramSocket()
            if (network != null) {
                try {
                    // The Ethernet network ROS started on (NetworkGate); without this the query
                    // would follow the default network, which need not be the Jetson link.
                    network.bindSocket(socket)
                } catch (e: IOException) {
                    Log.w(TAG, "could not bind the clock socket to $network; using the default network", e)
                }
            }
            socket.soTimeout = TIMEOUT_MS
            val sendPacket = DatagramPacket(request, request.size, address, NTP_PORT)
            val receivePacket = DatagramPacket(response, response.size)
            while (true) {
                polls++
                try {
                    poll(socket, sendPacket, receivePacket)
                } catch (e: SocketTimeoutException) {
                    timeouts++
                } catch (e: IOException) {
                    errors++
                    if (errors == 1L || errors % 60 == 0L) Log.e(TAG, "clock query to $server failed ($errors so far)", e)
                }
                checkStale()
                report()
                Thread.sleep(if (polls < BURST) BURST_INTERVAL_MS else INTERVAL_MS)
            }
        }

        private fun poll(socket: DatagramSocket, sendPacket: DatagramPacket, receivePacket: DatagramPacket) {
            val wall = measureWallMinusElapsedNs()
            if (answers > 0 && Math.abs(wall - wallMinusElapsedNs) > WALL_STEP_NS) {
                Log.w(TAG, String.format("Loomo wall clock stepped by %+.3f ms", (wall - wallMinusElapsedNs) / 1e6))
            }
            wallMinusElapsedNs = wall

            java.util.Arrays.fill(request, 0.toByte())
            request[0] = 0x23 // LI 0, version 4, mode 3 (client)
            val token = nonce.nextLong()
            ByteBuffer.wrap(request).putLong(40, token) // transmit timestamp: echoed back as originate
            val t1 = SystemClock.elapsedRealtimeNanos()
            socket.send(sendPacket)
            val deadline = t1 + TIMEOUT_MS * 1_000_000L
            var t4: Long
            val reply: ByteBuffer
            while (true) {
                receivePacket.length = response.size
                socket.receive(receivePacket) // SocketTimeoutException ends the poll
                t4 = SystemClock.elapsedRealtimeNanos()
                val b = ByteBuffer.wrap(response, 0, receivePacket.length)
                // Server mode and our token: not a late answer to an earlier query.
                if (receivePacket.length >= 48 && (response[0].toInt() and 0x7) == 4 && b.getLong(24) == token) {
                    reply = b
                    break
                }
                if (t4 > deadline) throw SocketTimeoutException()
            }
            val t2 = ntpToUnixNs(reply, 32)
            val t3 = ntpToUnixNs(reply, 40)
            // Standard NTP on-wire arithmetic, with the local times on elapsed realtime.
            val rtt = (t4 - t1) - (t3 - t2)
            jetsonLeap = (response[0].toInt() shr 6) and 0x3
            add(OffsetFilter.Sample(t4, ((t2 - t1) + (t3 - t4)) / 2, rtt))
            answers++
            lastAnswerElapsedNs = t4
            reportRtts.add(rtt)
        }

        private fun add(s: OffsetFilter.Sample) {
            val step = filter.add(s)
            if (step != null) {
                steps++
                Log.w(TAG, String.format("Jetson clock stepped by %+.3f ms; restarting the estimate", step / 1e6))
            }
            val estimate = filter.estimate()!!
            val newOffset = estimate.offsetNs - wallMinusElapsedNs
            offsetNs = newOffset
            if (!ready) {
                ready = true
                reportAtElapsedNs = s.atNs
                reportOffsetToElapsedNs = estimate.offsetNs
                Log.i(TAG, String.format(
                    "first clock sample after %.0f ms: Jetson - Loomo = %+.3f ms (rtt %.3f ms); publishing Jetson-time stamps (%d stamped messages dropped before)",
                    (s.atNs - startedAtElapsedNs) / 1e6, newOffset / 1e6, s.rttNs / 1e6, unsyncedDrops.get()))
            }
        }

        private fun checkStale() {
            val now = SystemClock.elapsedRealtimeNanos()
            if (!ready) {
                if (now - lastStaleLogElapsedNs > STALE_LOG_MS * 1_000_000L && now - startedAtElapsedNs > 5 * NS_PER_S) {
                    lastStaleLogElapsedNs = now
                    Log.e(TAG, "no answer from the Jetson's time server $server after ${(now - startedAtElapsedNs) / NS_PER_S} s " +
                        "($polls queries, $timeouts timeouts, $errors errors): stamped messages are NOT published " +
                        "(${unsyncedDrops.get()} dropped). Is loomo-sntp running on the Jetson?")
                }
                return
            }
            if (now - lastAnswerElapsedNs > STALE_LOG_MS * 1_000_000L && now - lastStaleLogElapsedNs > STALE_LOG_MS * 1_000_000L) {
                lastStaleLogElapsedNs = now
                Log.e(TAG, "no clock answer for ${(now - lastAnswerElapsedNs) / NS_PER_S} s; stamps keep the last offset and drift")
            }
        }

        private fun report() {
            if (!ready) return
            val now = SystemClock.elapsedRealtimeNanos()
            if (now - reportAtElapsedNs < REPORT_PERIOD_MS * 1_000_000L) return
            val estimate = filter.estimate() ?: return
            val drift = (estimate.offsetNs - reportOffsetToElapsedNs).toDouble() / (now - reportAtElapsedNs) * 1e6
            reportRtts.sort()
            val median = if (reportRtts.isEmpty()) 0L else reportRtts[reportRtts.size / 2]
            val max = if (reportRtts.isEmpty()) 0L else reportRtts[reportRtts.size - 1]
            val ages = AGES.mapNotNull { it.takeReport() }.joinToString("; ")
            Log.i(TAG, String.format(
                "offset Jetson - Loomo = %+.3f ms, estimate rtt %.3f ms (age %.0f s); rtt median %.3f max %.3f ms over %d answers; " +
                    "drift %+.2f ppm; polls %d answers %d timeouts %d errors %d steps %d edge failures %d; Jetson leap %d; unsynced drops %d; stamp ages: %s",
                offsetNs / 1e6, estimate.rttNs / 1e6, (now - estimate.atNs) / 1e9, median / 1e6, max / 1e6, reportRtts.size,
                drift, polls, answers, timeouts, errors, steps, edgeFailures, jetsonLeap, unsyncedDrops.get(), ages))
            reportRtts.clear()
            reportAtElapsedNs = now
            reportOffsetToElapsedNs = estimate.offsetNs
        }

        private fun ntpToUnixNs(b: ByteBuffer, at: Int): Long =
            OffsetFilter.ntpToUnixNs(b.getInt(at).toLong() and 0xFFFFFFFFL, b.getInt(at + 4).toLong() and 0xFFFFFFFFL)

        /**
         * Loomo wall clock minus elapsed realtime, to a few microseconds: currentTimeMillis()
         * has millisecond resolution, but at the instant it ticks over the wall clock is exactly
         * the new value. Each read is bracketed by two elapsed-time reads, and an edge counts only
         * if the bracket from the last read before it to the first read after it is narrow: a
         * thread preempted inside the loop otherwise measured 1.2 ms off (seen on 2026-10-13). With a
         * 20 us limit all attempts failed about once in 6 minutes; 100 us bounds the error to 50 us.
         * Spins for about 1 ms per attempt; if no attempt is clean the previous mapping is kept.
         */
        private fun measureWallMinusElapsedNs(): Long {
            var before = SystemClock.elapsedRealtimeNanos()
            var m0 = System.currentTimeMillis()
            repeat(EDGE_ATTEMPTS) {
                while (true) {
                    val e1 = SystemClock.elapsedRealtimeNanos()
                    val m = System.currentTimeMillis()
                    val e2 = SystemClock.elapsedRealtimeNanos()
                    if (m != m0) {
                        // The tick happened after the previous read (after `before`) and before
                        // this one (before e2).
                        if (e2 - before <= EDGE_MAX_BRACKET_NS) {
                            return m * 1_000_000L - (before + e2) / 2
                        }
                        m0 = m
                        before = e1
                        break
                    }
                    before = e1
                }
            }
            edgeFailures++
            return wallMinusElapsedNs
        }
    }
}
