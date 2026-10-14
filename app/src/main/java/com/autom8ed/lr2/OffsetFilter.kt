package com.autom8ed.lr2

import java.util.ArrayDeque

/**
 * The clock offset estimate of [TimeSync], kept free of Android and sockets so it can be unit
 * tested: the sample with the smallest round trip among the last [window] wins (its offset error is
 * at most half its round trip). A sample that disagrees with that estimate by more than both round
 * trips allow plus [stepMarginNs] is suspect and held aside, so a single bad answer never becomes
 * the estimate. [stepConfirm] suspects in a row mean the server's clock stepped: they replace the
 * window, so the estimate follows at once instead of holding the old clock for up to [window] polls.
 */
class OffsetFilter(
    private val window: Int = 16,
    private val stepMarginNs: Long = 5_000_000L,
    private val stepConfirm: Int = 3,
) {
    /** One SNTP exchange: server time minus local time ([offsetNs]) and its round trip. */
    class Sample(val atNs: Long, val offsetNs: Long, val rttNs: Long)

    private val samples = ArrayDeque<Sample>()
    private val suspects = ArrayList<Sample>()

    /** The current estimate, or null before the first sample. */
    fun estimate(): Sample? = samples.minByOrNull { it.rttNs }

    /** Adds [s]; returns the size of a server clock step (ns) if this sample confirmed one, else null. */
    fun add(s: Sample): Long? {
        val best = estimate()
        if (best != null && Math.abs(s.offsetNs - best.offsetNs) > (s.rttNs + best.rttNs) / 2 + stepMarginNs) {
            suspects.add(s)
            if (suspects.size < stepConfirm) return null
            samples.clear()
            samples.addAll(suspects)
            suspects.clear()
            return s.offsetNs - best.offsetNs
        }
        suspects.clear()
        samples.addLast(s)
        while (samples.size > window) samples.removeFirst()
        return null
    }

    companion object {
        private const val NTP_EPOCH_OFFSET_S = 2208988800L
        private const val NS_PER_S = 1_000_000_000L

        /** An NTP timestamp (unsigned 32-bit seconds since 1900 and fraction) in Unix nanoseconds. */
        fun ntpToUnixNs(seconds: Long, fraction: Long): Long =
            (seconds - NTP_EPOCH_OFFSET_S) * NS_PER_S + ((fraction * NS_PER_S) ushr 32)
    }
}
