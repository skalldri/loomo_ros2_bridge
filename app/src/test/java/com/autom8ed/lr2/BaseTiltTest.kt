package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class BaseTiltTest {
    /** What Segway's service puts in BASE_IMU's floatData for a controller count of 0.01 degree. */
    private fun sdkValue(counts: Int): Float = (counts * PI / 32767.0).toFloat()

    /** The true angle of [counts] hundredths of a degree, in radians. */
    private fun trueRad(counts: Int): Double = counts * PI / 18000.0

    @Test
    fun pitchRecoversTheTrueAngleFromTheServicesScale() {
        for (c in listOf(-9000, -2700, -1000, -1, 0, 1, 500, 1000, 2700, 9000)) {
            assertEquals("counts=$c", trueRad(c), BaseTilt.pitchRad(sdkValue(c)), 1e-6)
        }
    }

    @Test
    fun tenDegreesReadAsFivePointFiveAreTenAgain() {
        // 10 degrees is 1000 counts; the service reports 1000 * PI / 32767 = 0.0959 rad (5.49 deg).
        val sdk = sdkValue(1000)
        assertEquals(0.095877, sdk.toDouble(), 1e-6)
        assertEquals(Math.toRadians(10.0), BaseTilt.pitchRad(sdk), 1e-6)
    }

    @Test
    fun theFactorUndoesThe0_549Scale() {
        assertEquals(1.82039, BaseTilt.SDK_TO_TRUE, 1e-5)
        assertEquals(1.0, BaseTilt.SDK_TO_TRUE * (18000.0 / 32767.0), 1e-12)
    }

    @Test
    fun rollHasTheSameScaleAndTheOppositeSign() {
        for (c in listOf(-2700, -500, 0, 500, 2700)) {
            assertEquals("counts=$c", -trueRad(c), BaseTilt.rollRad(sdkValue(c)), 1e-6)
        }
        assertTrue(BaseTilt.rollRad(0.1f) < 0.0)
        assertTrue(BaseTilt.rollRad(-0.1f) > 0.0)
        assertTrue(BaseTilt.pitchRad(0.1f) > 0.0)
        assertTrue(BaseTilt.pitchRad(-0.1f) < 0.0)
    }

    @Test
    fun pitchOnlyIsARotationAboutY() {
        val p = 0.3
        val q = BaseTilt.quaternion(0.0, p)
        assertArrayEquals(doubleArrayOf(0.0, sin(p / 2), 0.0, cos(p / 2)), q)
    }

    @Test
    fun rollOnlyIsARotationAboutX() {
        val r = -0.2
        val q = BaseTilt.quaternion(r, 0.0)
        assertArrayEquals(doubleArrayOf(sin(r / 2), 0.0, 0.0, cos(r / 2)), q)
    }

    @Test
    fun positivePitchLeansTheTopForwardAndPositiveRollLeansItRight() {
        // REP 103: x forward, y left, z up. base_link's z axis expressed in base_odom.
        val up = 0.1
        val zPitched = rotate(BaseTilt.quaternion(0.0, up), doubleArrayOf(0.0, 0.0, 1.0))
        assertTrue(zPitched[0] > 0.0) // top forward, nose (x) down
        assertEquals(sin(up), zPitched[0], 1e-12)
        val zRolled = rotate(BaseTilt.quaternion(up, 0.0), doubleArrayOf(0.0, 0.0, 1.0))
        assertTrue(zRolled[1] < 0.0) // top toward -y (right), left side up
    }

    @Test
    fun combinedTiltIsAUnitQuaternionThatReturnsBothAngles() {
        for (r in listOf(-0.4, -0.05, 0.0, 0.1, 0.3)) {
            for (p in listOf(-0.5, -0.1, 0.0, 0.2, 0.45)) {
                val q = BaseTilt.quaternion(r, p)
                assertEquals(1.0, sqrt(q.sumOf { it * it }), 1e-12)
                val (x, y, z, w) = q.toList()
                val roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
                val pitch = Math.asin(2 * (w * y - z * x))
                val yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
                assertEquals(r, roll, 1e-12)
                assertEquals(p, pitch, 1e-12)
                assertEquals(0.0, yaw, 1e-12)
            }
        }
    }

    @Test
    fun matchesTheSdksSetEulerRadUsedBefore() {
        // The transform used to be built with the SDK's Quaternion.setEulerRad(yaw, pitch, roll).
        for (r in listOf(-0.3, 0.0, 0.25)) {
            for (p in listOf(-0.4, 0.0, 0.15)) {
                val sdk = com.segway.robot.algo.tf.Quaternion(0f, 0f, 0f, 0f)
                sdk.setEulerRad(0f, p.toFloat(), r.toFloat())
                val q = BaseTilt.quaternion(r, p)
                assertEquals(sdk.x.toDouble(), q[0], 1e-6)
                assertEquals(sdk.y.toDouble(), q[1], 1e-6)
                assertEquals(sdk.z.toDouble(), q[2], 1e-6)
                assertEquals(sdk.w.toDouble(), q[3], 1e-6)
            }
        }
    }

    private fun assertArrayEquals(expected: DoubleArray, actual: DoubleArray) {
        assertEquals(expected.size, actual.size)
        for (i in expected.indices) assertEquals("[$i]", expected[i], actual[i], 1e-12)
    }

    /** [v] rotated by the unit quaternion [q] (x, y, z, w). */
    private fun rotate(q: DoubleArray, v: DoubleArray): DoubleArray {
        val (x, y, z, w) = q.toList()
        // v' = v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
        val cx = y * v[2] - z * v[1]
        val cy = z * v[0] - x * v[2]
        val cz = x * v[1] - y * v[0]
        return doubleArrayOf(
            v[0] + 2 * w * cx + 2 * (y * cz - z * cy),
            v[1] + 2 * w * cy + 2 * (z * cx - x * cz),
            v[2] + 2 * w * cz + 2 * (x * cy - y * cx),
        )
    }
}
