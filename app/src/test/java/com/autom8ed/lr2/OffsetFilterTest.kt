package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Test

class OffsetFilterTest {
    private val ms = 1_000_000L

    private fun sample(i: Int, offsetMs: Double, rttMs: Double) =
        OffsetFilter.Sample(i * 1000 * ms, (offsetMs * ms).toLong(), (rttMs * ms).toLong())

    @Test
    fun picksTheSmallestRoundTrip() {
        val f = OffsetFilter(window = 16)
        assertNull(f.estimate())
        f.add(sample(0, 10.4, 0.8))
        f.add(sample(1, 10.1, 0.3))
        f.add(sample(2, 12.0, 4.0)) // a slow exchange: within its own uncertainty, not a step
        assertEquals(10.1 * ms, f.estimate()!!.offsetNs.toDouble(), 1.0)
    }

    @Test
    fun oldSamplesLeaveTheWindow() {
        val f = OffsetFilter(window = 4)
        f.add(sample(0, 10.0, 0.1))
        for (i in 1..4) f.add(sample(i, 10.2, 0.5))
        assertEquals(10.2 * ms, f.estimate()!!.offsetNs.toDouble(), 1.0)
    }

    @Test
    fun oneOutlierIsNotAStep() {
        val f = OffsetFilter(window = 16, stepMarginNs = 5 * ms, stepConfirm = 3)
        for (i in 0..4) assertNull(f.add(sample(i, 0.0, 0.5)))
        assertNull(f.add(sample(5, 50.0, 0.4))) // one bad answer
        assertNull(f.add(sample(6, 0.1, 0.6)))
        assertNull(f.add(sample(7, 50.0, 0.4)))
        // The outliers have the smallest round trip, but suspects never become the estimate.
        assertEquals(0.0, f.estimate()!!.offsetNs.toDouble(), 1.0)
    }

    @Test
    fun aConfirmedStepResetsTheEstimate() {
        val f = OffsetFilter(window = 16, stepMarginNs = 5 * ms, stepConfirm = 3)
        for (i in 0..9) f.add(sample(i, 0.0, 0.2)) // very good samples on the old clock
        assertNull(f.add(sample(10, 2000.0, 0.6)))
        assertNull(f.add(sample(11, 2000.1, 0.7)))
        val step = f.add(sample(12, 2000.0, 0.5))
        assertNotNull(step)
        assertEquals(2000.0 * ms, step!!.toDouble(), 0.2 * ms)
        // Without the reset the old 0.2 ms samples would win for another 13 polls.
        assertEquals(2000.0 * ms, f.estimate()!!.offsetNs.toDouble(), 0.2 * ms)
    }

    @Test
    fun ntpTimestamps() {
        // 2026-10-13T00:00:00Z = 1791849600 Unix seconds; fraction 0x80000000 = 0.5 s.
        val unixS = 1791849600L
        assertEquals(unixS * 1_000_000_000L + 500_000_000L, OffsetFilter.ntpToUnixNs(unixS + 2208988800L, 0x80000000L))
        // Largest fraction: just under one second, no overflow.
        assertEquals(unixS * 1_000_000_000L + 999_999_999L, OffsetFilter.ntpToUnixNs(unixS + 2208988800L, 0xFFFFFFFFL))
    }
}
