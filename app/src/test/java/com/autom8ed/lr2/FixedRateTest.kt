package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class FixedRateTest {
    private val ms = 1_000_000L

    @Test
    fun ticksEveryPeriodFromTheStart() {
        val t = FixedRate(50 * ms, startNs = 0L)
        assertTrue(t.due(0L))
        assertFalse(t.due(49 * ms))
        assertTrue(t.due(50 * ms))
        assertFalse(t.due(50 * ms))
        assertTrue(t.due(100 * ms))
        assertEquals(0L, t.skipped)
    }

    @Test
    fun waitNsCountsDownToTheNextDeadline() {
        val t = FixedRate(50 * ms, startNs = 0L)
        assertEquals(0L, t.waitNs(0L))
        assertTrue(t.due(0L))
        assertEquals(50 * ms, t.waitNs(0L))
        assertEquals(20 * ms, t.waitNs(30 * ms))
        assertEquals(0L, t.waitNs(60 * ms))
    }

    @Test
    fun aLateTickKeepsThePhase() {
        val t = FixedRate(50 * ms, startNs = 0L)
        assertTrue(t.due(0L))
        assertTrue(t.due(70 * ms)) // 20 ms late
        assertEquals(30 * ms, t.waitNs(70 * ms)) // next is still at 100 ms
        assertFalse(t.due(99 * ms))
        assertTrue(t.due(100 * ms))
        assertEquals(0L, t.skipped)
    }

    @Test
    fun aStallSkipsTheMissedTicksInsteadOfBursting() {
        val t = FixedRate(50 * ms, startNs = 0L)
        assertTrue(t.due(0L))
        assertTrue(t.due(175 * ms)) // deadlines 50, 100 and 150 were missed: one tick now
        assertEquals(2L, t.skipped) // 100 and 150 skipped; 50 is served by this tick
        assertFalse(t.due(176 * ms))
        assertEquals(25 * ms, t.waitNs(175 * ms)) // back on phase at 200 ms
        assertTrue(t.due(200 * ms))
    }

    @Test
    fun aLoopWakingOnFramesAndTimeoutsTicksAtTwentyHertz() {
        // The tf-publisher loop wakes for 30 Hz frames and for the timer; with frames, without
        // frames (a stalled depth stream) and with both, the timer fires 20 times a second.
        for (frames in listOf(true, false)) {
            val t = FixedRate(50 * ms, startNs = 0L)
            var now = 0L
            var ticks = 0
            var nextFrame = 0L
            while (now < 1000 * ms) {
                val wait = t.waitNs(now)
                now = if (frames && nextFrame < now + wait) maxOf(now, nextFrame) else now + wait
                if (frames && now >= nextFrame) nextFrame += 33_333_333L
                now += 2 * ms // the work itself, late on every wake
                if (t.due(now) && now < 1000 * ms) ticks++
            }
            assertEquals("frames=$frames", 20, ticks)
            assertEquals(0L, t.skipped)
        }
    }

    @Test(expected = IllegalArgumentException::class)
    fun rejectsANonPositivePeriod() {
        FixedRate(0L, startNs = 0L)
    }
}
