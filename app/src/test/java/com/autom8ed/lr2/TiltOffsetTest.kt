package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

class TiltOffsetTest {
    /** What Segway's service puts in BASE_IMU's floatData for a controller count of 0.01 degree. */
    private fun sdkValue(counts: Int): Float = (counts * PI / 32767.0).toFloat()

    private fun deg(rad: Double) = Math.toDegrees(rad)

    @Test
    fun theDefaultChangesNothing() {
        val o = TiltOffset()
        for (a in listOf(-0.3, -0.01, 0.0, 0.02, 0.4)) {
            assertEquals(a, o.roll(a), 0.0)
            assertEquals(a, o.pitch(a), 0.0)
        }
    }

    @Test
    fun aLevelBaseReadsZero() {
        // Measured on 2026-10-24: level per a digital level, the app read +0.89 degree of roll.
        // That is 89 counts with the ROS sign; BASE_IMU's roll has the opposite sign.
        val o = TiltOffset(rollDeg = 0.89)
        val roll = o.roll(BaseTilt.rollRad(sdkValue(-89)))
        assertEquals(0.0, deg(roll), 1e-4)
    }

    @Test
    fun theSecondSpotMatchesTheLevel() {
        // Same day: the level read 1.4 degree, right side lower (ROS roll +1.4); the app read +2.27.
        val o = TiltOffset(rollDeg = 0.88)
        assertEquals(1.39, deg(o.roll(BaseTilt.rollRad(sdkValue(-227)))), 1e-4)
    }

    @Test
    fun offsetsAreSubtractedInDegreesWithTheirSigns() {
        val o = TiltOffset(rollDeg = 0.88, pitchDeg = -0.5)
        assertEquals(2.0 - 0.88, deg(o.roll(Math.toRadians(2.0))), 1e-12)
        assertEquals(-3.0 + 0.5, deg(o.pitch(Math.toRadians(-3.0))), 1e-12)
        // Each offset touches only its own axis.
        assertEquals(1.0, deg(TiltOffset(rollDeg = 2.0).pitch(Math.toRadians(1.0))), 1e-12)
        assertEquals(1.0, deg(TiltOffset(pitchDeg = 2.0).roll(Math.toRadians(1.0))), 1e-12)
    }

    @Test
    fun aRollOffsetIsExactForASensorRolledOnTheBase() {
        // The controller's sensor rolled by r0 on the base: it reports Ry(p) Rx(r + r0) for a base
        // at Ry(p) Rx(r). Removing r0 must give the base's own rotation, for any pitch.
        val r0 = Math.toRadians(0.88)
        val o = TiltOffset(rollDeg = 0.88)
        for (p in listOf(-0.3, -0.1, 0.0, 0.15, 0.35)) {
            for (r in listOf(-0.1, 0.0, 0.05)) {
                val q = BaseTilt.quaternion(o.roll(r + r0), o.pitch(p))
                assertQuaternionEquals(BaseTilt.quaternion(r, p), q)
                // The same as composing the measured rotation with Rx(-r0) on the right.
                val composed = multiply(BaseTilt.quaternion(r + r0, p), axisX(-r0))
                assertQuaternionEquals(composed, q)
            }
        }
    }

    @Test
    fun aPitchOffsetIsExactForAnOffsetLevelReference() {
        // The controller's level reference pitched by p0: Ry(p0) Ry(p) Rx(r) for a base at Ry(p) Rx(r).
        val p0 = Math.toRadians(-0.6)
        val o = TiltOffset(pitchDeg = -0.6)
        for (p in listOf(-0.3, 0.0, 0.2)) {
            for (r in listOf(-0.1, 0.0, 0.07)) {
                val measured = multiply(axisY(p0), BaseTilt.quaternion(r, p))
                val q = BaseTilt.quaternion(o.roll(r), o.pitch(p + p0))
                assertQuaternionEquals(multiply(axisY(-p0), measured), q)
                assertQuaternionEquals(BaseTilt.quaternion(r, p), q)
            }
        }
    }

    @Test
    fun theLimitIsFiveDegrees() {
        assertEquals(5.0, TiltOffset.MAX_DEG, 0.0)
        assertTrue(TiltOffset(rollDeg = 0.88).toString().contains("tilt.roll_offset_deg=0.88"))
    }

    private fun axisX(a: Double) = doubleArrayOf(sin(a / 2), 0.0, 0.0, cos(a / 2))
    private fun axisY(a: Double) = doubleArrayOf(0.0, sin(a / 2), 0.0, cos(a / 2))

    /** The Hamilton product [a] * [b] of quaternions stored as x, y, z, w. */
    private fun multiply(a: DoubleArray, b: DoubleArray): DoubleArray {
        val (ax, ay, az, aw) = a.toList()
        val (bx, by, bz, bw) = b.toList()
        return doubleArrayOf(
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
    }

    /** Equal as rotations: q and -q are the same rotation. */
    private fun assertQuaternionEquals(expected: DoubleArray, actual: DoubleArray) {
        val sign = if (expected.indices.sumOf { expected[it] * actual[it] } < 0) -1.0 else 1.0
        for (i in expected.indices) assertEquals("[$i]", expected[i], sign * actual[i], 1e-12)
    }
}
