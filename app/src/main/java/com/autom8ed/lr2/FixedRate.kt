package com.autom8ed.lr2

/**
 * The deadlines of a fixed-rate timer on a monotonic clock, for a loop that also waits for other
 * work (TfPublisher's thread waits for depth frames with [waitNs] as its timeout).
 *
 * Deadlines stay on the phase set by [startNs]: a late tick does not shift the ones after it.
 * After a stall longer than a period the missed deadlines are skipped and counted rather than
 * run back to back.
 */
class FixedRate(private val periodNs: Long, startNs: Long) {
    init {
        require(periodNs > 0) { "periodNs must be positive" }
    }

    private var nextNs = startNs

    /** Deadlines skipped after stalls, since start. */
    var skipped = 0L
        private set

    /** Nanoseconds until the next deadline; 0 if it is due. */
    fun waitNs(nowNs: Long): Long = maxOf(0L, nextNs - nowNs)

    /** True if a deadline is due at [nowNs]; then moves on to the next deadline after [nowNs]. */
    fun due(nowNs: Long): Boolean {
        if (nowNs < nextNs) return false
        val missed = (nowNs - nextNs) / periodNs
        skipped += missed
        nextNs += (missed + 1) * periodNs
        return true
    }
}
