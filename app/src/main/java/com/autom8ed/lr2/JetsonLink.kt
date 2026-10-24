package com.autom8ed.lr2

/**
 * The speed of the Ethernet link between the Loomo and the Jetson, as the Jetson reports it on
 * /loomo/jetson_link (std_msgs/UInt16, Mb/s, 0 without carrier, once a second). The Loomo cannot
 * read it itself: its USB Ethernet adapter runs under cdc_ether, which gives EINVAL for
 * /sys/class/net/eth0/speed.
 *
 * The link sometimes comes up at 100 Mb/s instead of 1000, and the camera streams (~14 MB/s) do
 * not fit, so frames drop. [warning] gives the text of the screen's banner while the reported
 * speed is below [fullMbps], and null otherwise: at full speed, before the first report, after
 * [staleNs] without one (the speed is then unknown, and a stale value must not raise or keep a
 * warning), and for 0 (no carrier on the Jetson: the report cannot have come over that link).
 *
 * Plain Kotlin with the clock passed in, so the JVM unit tests drive it. [onReport] is called on
 * the ROS executor's threads and [status] and [warning] on the UI thread.
 */
class JetsonLink(
    val staleNs: Long = STALE_NS,
    val fullMbps: Int = FULL_MBPS,
) {
    enum class State {
        /** No report yet, the last one is older than [staleNs], or the Jetson reports no carrier. */
        UNKNOWN,
        /** At or above [fullMbps]. */
        FULL,
        /** Above 0 and below [fullMbps]: camera frames drop. */
        SLOW,
    }

    /** [mbps] is the last report while it is fresh, null when there is none. */
    data class Status(val state: State, val mbps: Int?)

    private class Report(val mbps: Int, val atNs: Long)

    @Volatile
    private var last: Report? = null

    /**
     * Records a report received at [nowNs] (a monotonic clock, System.nanoTime() in the app).
     * Returns true when the speed differs from the previous report, or there was none, so the
     * caller can log changes.
     */
    fun onReport(mbps: Int, nowNs: Long): Boolean {
        val previous = last
        last = Report(mbps, nowNs)
        return previous == null || previous.mbps != mbps
    }

    fun status(nowNs: Long): Status {
        val report = last ?: return Status(State.UNKNOWN, null)
        if (nowNs - report.atNs > staleNs) return Status(State.UNKNOWN, null)
        val state = when {
            report.mbps <= 0 -> State.UNKNOWN
            report.mbps >= fullMbps -> State.FULL
            else -> State.SLOW
        }
        return Status(state, report.mbps)
    }

    /** The banner's text while the link is slow; null when there is nothing to warn about. */
    fun warning(nowNs: Long): String? {
        val status = status(nowNs)
        return if (status.state == State.SLOW) warningText(status.mbps!!) else null
    }

    /** For StreamStats: the last report, its age, and the state the screen shows. */
    fun statsLine(nowNs: Long): String {
        val report = last
        val status = status(nowNs)
        return if (report == null) {
            "jetson_link state=${status.state} no report"
        } else {
            "jetson_link state=${status.state} last=${report.mbps} Mb/s age=${(nowNs - report.atNs) / 1_000_000} ms"
        }
    }

    companion object {
        const val TOPIC = "/loomo/jetson_link"
        const val FULL_MBPS = 1000

        /** The Jetson reports every second; more than 5 s of silence means the speed is unknown. */
        const val STALE_NS = 5_000_000_000L

        /** std_msgs/UInt16 arrives as a Java short: read it unsigned. */
        fun mbpsOf(data: Short): Int = data.toInt() and 0xFFFF

        fun warningText(mbps: Int): String =
            "Ethernet to the Jetson at $mbps Mb/s: camera frames will drop. " +
                "Reseat the cable at both ends."
    }
}
