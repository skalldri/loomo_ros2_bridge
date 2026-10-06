package com.autom8ed.lr2

import android.util.Log
import com.autom8ed.lr2.vision.StreamStats
import java.util.concurrent.CopyOnWriteArrayList

/**
 * Logs one accounting line per camera stream every [periodMs] under the tag "StreamStats":
 * cumulative counters, the change since the previous line (+n) and rates, plus the window
 * maxima. Nothing is published on ROS; the Jetson side verifies delivery by counting what it
 * receives over the same interval (see the frame probe) and diffing against these numbers.
 */
class StatsReporter(private val periodMs: Long = 5000) {
    private val streams = CopyOnWriteArrayList<StreamStats>()
    private val extras = CopyOnWriteArrayList<() -> String>()
    private val previous = HashMap<String, StreamStats.Snapshot>()
    @Volatile private var running = false
    private var thread: Thread? = null

    fun register(stats: StreamStats) { streams.add(stats) }

    /** A supplier for one extra line per report (e.g. the TF queue). */
    fun registerExtra(supplier: () -> String) { extras.add(supplier) }

    fun start() {
        if (running) return
        running = true
        thread = Thread({ loop() }, "stats-reporter").apply { isDaemon = true; start() }
    }

    fun stop() {
        running = false
        thread?.interrupt()
        thread = null
    }

    private fun loop() {
        var lastAt = System.nanoTime()
        while (running) {
            try {
                Thread.sleep(periodMs)
            } catch (e: InterruptedException) {
                return
            }
            val now = System.nanoTime()
            val windowS = (now - lastAt) / 1e9
            lastAt = now
            for (s in streams) {
                val snap = s.snapshot()
                s.resetWindowMaxima()
                val prev = previous[s.name]
                previous[s.name] = snap
                Log.i(StreamStats.TAG, format(s.name, snap, prev, windowS))
            }
            for (e in extras) {
                try {
                    Log.i(StreamStats.TAG, e())
                } catch (t: Throwable) {
                    Log.w(StreamStats.TAG, "extra reporter failed", t)
                }
            }
        }
    }

    private fun format(name: String, s: StreamStats.Snapshot, p: StreamStats.Snapshot?, windowS: Double): String {
        fun d(cur: Long, prev: Long?): String = if (prev == null) "$cur" else "$cur(+${cur - prev})"
        val seenDelta = if (p == null) 0 else s.sdkFramesSeen - p.sdkFramesSeen
        val sb = StringBuilder()
        sb.append(name)
            .append(" seen=").append(d(s.sdkFramesSeen, p?.sdkFramesSeen))
            .append(String.format(" %.1f/s", seenDelta / windowS))
            .append(" svc_gap=").append(d(s.sdkGapFrames, p?.sdkGapFrames))
            .append(" regress=").append(d(s.sdkFrameNumRegressions, p?.sdkFrameNumRegressions))
            .append(" queued=").append(d(s.queued, p?.queued))
            .append(" q_drop=").append(d(s.queueDropped, p?.queueDropped))
            .append(" rate_skip=").append(d(s.rateSkipped, p?.rateSkipped))
            .append(" fail=").append(d(s.publishFailures, p?.publishFailures))
            .append(" enc_restart=").append(d(s.encoderRestarts, p?.encoderRestarts))
        for ((topic, t) in s.topics.toSortedMap()) {
            val pt = p?.topics?.get(topic)
            sb.append(" pub[").append(shortTopic(topic)).append("]=")
                .append(d(t.first, pt?.first))
                .append(" nosub=").append(d(t.second, pt?.second))
        }
        sb.append(String.format(" cb_max=%.1fms wk_max=%.1fms q_max=%d", s.callbackNsMax / 1e6, s.workerNsMax / 1e6, s.queueDepthMax))
            .append(" last_frame=").append(s.lastFrameNum)
        return sb.toString()
    }

    private fun shortTopic(topic: String): String {
        // "/loomo/realsense/depth/image_depth_rect" -> "image_depth_rect", keep a transport suffix
        val parts = topic.trim('/').split('/')
        return if (parts.size >= 2 && parts.last() in setOf("h264", "compressed", "compressedDepth", "camera_info"))
            parts[parts.size - 2] + "/" + parts.last() else parts.last()
    }
}
