package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.floor
import kotlin.math.sin

class SampleGateTest {
    @Test
    fun aThirtyHertzCaptureOfATwentyHertzSourcePassesEachSampleOnce() {
        // Captures at depth frame times (33.3 ms apart) see the latest 20 Hz SDK snapshot.
        val g = SampleGate()
        val published = ArrayList<Long>()
        for (i in 0 until 90) {
            val captureUs = 1_000_000L + i * 33_333L
            val snapshotUs = (captureUs / 50_000L) * 50_000L
            if (g.isNew(snapshotUs)) published.add(snapshotUs)
        }
        assertEquals(60, published.size)
        assertEquals(published.distinct(), published)
        assertEquals(30L, g.repeats)
        assertEquals(60L, g.passed)
    }

    @Test
    fun repeatsAndSmallStepsBackAreRejected() {
        val g = SampleGate()
        assertTrue(g.isNew(1_000_000L))
        assertFalse(g.isNew(1_000_000L))
        assertTrue(g.isNew(1_050_000L))
        assertFalse(g.isNew(1_000_000L)) // an older snapshot: not new
        assertTrue(g.isNew(1_150_000L)) // a 100 ms interval is fine
    }

    @Test
    fun aClockStepBackIsFollowed() {
        val g = SampleGate(resetUs = 1_000_000L)
        assertTrue(g.isNew(100_000_000L))
        assertTrue(g.isNew(50_000_000L)) // 50 s back: the Loomo's clock was set back
        assertFalse(g.isNew(50_000_000L))
        assertTrue(g.isNew(50_050_000L))
    }
}

class PoseDiffTwistTest {
    private val tick = 0.0192666 // Pose2D yaw quantum (one encoder tick)
    private val step = 0.00445 // Pose2D translation quantum

    private fun q(v: Double, quantum: Double) = floor(v / quantum + 0.5) * quantum

    @Test
    fun fallsBackUntilTheWindowFills() {
        val t = PoseDiffTwist(windowUs = 250_000L)
        for (i in 0 until 5) assertNull(t.add(i * 50_000L, 0.0, 0.0, 0.0))
        assertNotNull(t.add(5 * 50_000L, 0.0, 0.0, 0.0))
    }

    @Test
    fun atRestTheTwistIsZero() {
        val t = PoseDiffTwist(windowUs = 250_000L)
        var last: PoseDiffTwist.Twist? = null
        for (i in 0 until 40) last = t.add(i * 50_000L, 1.234, -0.5, 2.0) ?: last
        assertEquals(0.0, last!!.linear, 1e-12)
        assertEquals(0.0, last.angular, 1e-12)
    }

    @Test
    fun straightLineAtAnAngleWithQuantisedPose() {
        // 0.5 m/s along a heading of 30 degrees; x/y quantised to 4.45 mm, yaw to one tick.
        val t = PoseDiffTwist(windowUs = 250_000L)
        val heading = q(PI / 6, tick)
        val got = ArrayList<PoseDiffTwist.Twist>()
        for (i in 0 until 100) {
            val s = 0.5 * i * 0.05
            val r = t.add(i * 50_000L, q(s * cos(heading), step), q(s * sin(heading), step), heading)
            if (r != null) got.add(r)
        }
        assertEquals(95, got.size)
        for (r in got) {
            // Two quantisation errors of at most half a step each over 0.25 s.
            assertEquals(0.5, r.linear, step / 0.25 * 1.5)
            assertEquals(0.0, r.angular, 1e-12)
        }
    }

    @Test
    fun turningInPlaceWithQuantisedYaw() {
        // 0.4 rad/s in place: a single-sample difference would be 0 or 0.385 rad/s, the window
        // is within half a tick per end of the truth.
        val t = PoseDiffTwist(windowUs = 250_000L)
        var n = 0
        for (i in 0 until 100) {
            val r = t.add(i * 50_000L, 0.0, 0.0, q(0.4 * i * 0.05, tick)) ?: continue
            assertEquals(0.4, r.angular, tick / 0.25 + 1e-9)
            assertEquals(0.0, r.linear, 1e-12)
            n++
        }
        assertEquals(95, n)
    }

    @Test
    fun aHundredMillisecondGapUsesTheActualInterval() {
        // Samples every 50 ms except one missing: the window then spans 300 ms, and the rate
        // must still be the true one (a fixed sample count would read 20% high).
        val t = PoseDiffTwist(windowUs = 250_000L)
        val times = (0..20).map { it * 50_000L }.filter { it != 500_000L }
        var last: PoseDiffTwist.Twist? = null
        // An arc: 0.3 m/s forward while turning at 0.2 rad/s (radius 1.5 m).
        for (tUs in times) {
            val th = 0.2 * tUs / 1e6
            val r = t.add(tUs, 1.5 * sin(th), 1.5 * (1 - cos(th)), th)
            if (r != null) {
                assertEquals(0.2, r.angular, 1e-9)
                // Arc versus chord on the mean heading: within 0.1% here.
                assertEquals(0.3, r.linear, 0.3e-3)
            }
            last = r ?: last
        }
        assertNotNull(last)
    }

    @Test
    fun wrapsAroundPlusMinusPi() {
        // Turning left at 1 rad/s through +pi: theta jumps from ~+pi to ~-pi.
        val t = PoseDiffTwist(windowUs = 250_000L)
        var n = 0
        for (i in 0 until 40) {
            val theta = PoseDiffTwist.wrap(PI - 0.5 + 1.0 * i * 0.05)
            val r = t.add(i * 50_000L, 0.0, 0.0, theta) ?: continue
            assertEquals(1.0, r.angular, 1e-9)
            n++
        }
        assertEquals(35, n)
        // And turning right through -pi.
        val u = PoseDiffTwist(windowUs = 250_000L)
        for (i in 0 until 40) {
            val theta = PoseDiffTwist.wrap(-PI + 0.5 - 1.0 * i * 0.05)
            val r = u.add(i * 50_000L, 0.0, 0.0, theta) ?: continue
            assertEquals(-1.0, r.angular, 1e-9)
        }
    }

    @Test
    fun forwardSpeedAcrossTheWrapUsesTheMeanHeading() {
        // Driving forward at 0.5 m/s while the heading crosses pi: the mean heading must be taken
        // across the wrap (pi), not the arithmetic mean of +3.1 and -3.1 (0).
        val t = PoseDiffTwist(windowUs = 250_000L)
        val r0 = t.add(0L, 0.0, 0.0, PI - 0.01)
        assertNull(r0)
        for (i in 1 until 5) t.add(i * 50_000L, -0.5 * i * 0.05, 0.0, PI - 0.01 + 0.004 * i)
        val r = t.add(250_000L, -0.125, 0.0, PoseDiffTwist.wrap(PI - 0.01 + 0.02))
        assertNotNull(r)
        assertEquals(0.5, r!!.linear, 1e-3)
        assertEquals(0.08, r.angular, 1e-9)
    }

    @Test
    fun aStallFallsBackToTheSdkUntilTheWindowRefills() {
        val t = PoseDiffTwist(windowUs = 250_000L, maxDtUs = 600_000L)
        for (i in 0..10) t.add(i * 50_000L, 0.0, 0.0, 0.0)
        assertNotNull(t.add(550_000L, 0.0, 0.0, 0.0))
        // Nothing for 1 s, then samples resume.
        val resume = 1_550_000L
        for (k in 0 until 5) assertNull(t.add(resume + k * 50_000L, 0.0, 0.0, 0.0))
        assertNotNull(t.add(resume + 5 * 50_000L, 0.0, 0.0, 0.0))
    }

    @Test
    fun aClockStepBackStartsOver() {
        val t = PoseDiffTwist(windowUs = 250_000L)
        for (i in 0..10) t.add(100_000_000L + i * 50_000L, 0.0, 0.0, 0.0)
        assertNull(t.add(50_000_000L, 5.0, 5.0, 1.0)) // no difference across the step
        for (k in 1 until 5) assertNull(t.add(50_000_000L + k * 50_000L, 5.0, 5.0, 1.0))
        val r = t.add(50_250_000L, 5.0, 5.0, 1.0)
        assertEquals(0.0, r!!.angular, 1e-12)
    }

    @Test
    fun wrap() {
        assertEquals(0.0, PoseDiffTwist.wrap(2 * PI), 1e-12)
        assertEquals(PI, PoseDiffTwist.wrap(-PI), 1e-12)
        assertEquals(-PI / 2, PoseDiffTwist.wrap(3 * PI / 2), 1e-12)
        assertEquals(0.1, PoseDiffTwist.wrap(0.1 - 4 * PI), 1e-12)
    }
}
