package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class HeadAttitudeTest {
    /** What Segway's service reports for a head controller count: `raw / 10427.832`. */
    private fun sdkValue(counts: Int): Float = counts / 10427.832f

    @Test
    fun theHeadsScaleIsUsedAsIs() {
        // 182 counts per degree: 1820 counts are 10 degrees, 16384 counts are 90.02 degrees.
        assertEquals(Math.toRadians(10.0), HeadAttitude.rollRad(sdkValue(1820)), 1e-6)
        assertEquals(Math.toRadians(-10.0), HeadAttitude.jointRollRad(sdkValue(-1820)), 1e-6)
        assertEquals(Math.toRadians(16384 / 182.0), HeadAttitude.yawRad(sdkValue(16384)), 1e-6)
        // Full scale is pi to 0.02%: no BASE_IMU-style 32767/18000 factor belongs here.
        assertEquals(PI, HeadAttitude.yawRad(sdkValue(32767)), PI * 3e-4)
        assertEquals(1.0, sdkValue(32767) / (32767 * PI / 32767.0), 3e-4)
    }

    @Test
    fun pitchIsNegatedRollYawAndJointRollAreKept() {
        for (v in listOf(-1.2f, -0.1f, 0f, 0.05f, 0.7f)) {
            assertEquals(-v.toDouble(), HeadAttitude.pitchRad(v), 0.0)
            assertEquals(v.toDouble(), HeadAttitude.rollRad(v), 0.0)
            assertEquals(v.toDouble(), HeadAttitude.yawRad(v), 0.0)
            assertEquals(v.toDouble(), HeadAttitude.jointRollRad(v), 0.0)
        }
    }

    @Test
    fun theJointPitchSeenParkedComesOutNoseDown() {
        // Parked on 2026-10-14 HEAD_JOINT_PITCH read -0.111 rad while Segway's TF put the same joint
        // at +0.112 rad about y (nose down). World pitch has the same raw convention, so that reading
        // must come out as a positive ROS pitch.
        assertEquals(0.111, HeadAttitude.pitchRad(-0.111f), 1e-6)
    }

    @Test
    fun aPositiveSdkPitchTiltsTheNoseUp() {
        // REP 103: x forward, y left, z up. The head's x axis in the world.
        val q = HeadAttitude.orientation(floatArrayOf(0.2f, 0f, 0f))
        val nose = rotate(q, doubleArrayOf(1.0, 0.0, 0.0))
        assertTrue(nose[2] > 0.0)
        assertEquals(sin(0.2), nose[2], 1e-6)
    }

    @Test
    fun aPositiveSdkRollPutsTheRightSideDown() {
        // ROS roll: left side up. The head's y axis (left) in the world rises.
        val q = HeadAttitude.orientation(floatArrayOf(0f, 0.1f, 0f))
        val left = rotate(q, doubleArrayOf(0.0, 1.0, 0.0))
        assertTrue(left[2] > 0.0)
        assertEquals(sin(0.1), left[2], 1e-6)
        // Its up axis leans to the right (-y).
        assertTrue(rotate(q, doubleArrayOf(0.0, 0.0, 1.0))[1] < 0.0)
    }

    @Test
    fun aPositiveSdkYawTurnsTheNoseLeft() {
        // ROS yaw: counterclockwise seen from above, so the nose swings toward +y.
        val q = HeadAttitude.orientation(floatArrayOf(0f, 0f, 0.3f))
        val nose = rotate(q, doubleArrayOf(1.0, 0.0, 0.0))
        assertTrue(nose[1] > 0.0)
        assertEquals(sin(0.3), nose[1], 1e-6)
    }

    @Test
    fun orientationTakesPitchRollYawFromTheirSlots() {
        val f = floatArrayOf(0.12f, -0.05f, 1.3f)
        val expected = HeadAttitude.quaternion(-0.05, -0.12, 1.3)
        assertArrayEquals(expected, HeadAttitude.orientation(f), 1e-7)
    }

    @Test
    fun isYawThenPitchThenRoll() {
        // setRPY(r, p, y) = Rz(y) * Ry(p) * Rx(r).
        for ((r, p, y) in listOf(Triple(0.1, -0.2, 0.3), Triple(-0.4, 0.5, -2.9), Triple(0.03, 1.2, 3.1))) {
            val qx = doubleArrayOf(sin(r / 2), 0.0, 0.0, cos(r / 2))
            val qy = doubleArrayOf(0.0, sin(p / 2), 0.0, cos(p / 2))
            val qz = doubleArrayOf(0.0, 0.0, sin(y / 2), cos(y / 2))
            assertArrayEquals(mul(mul(qz, qy), qx), HeadAttitude.quaternion(r, p, y), 1e-12)
        }
    }

    @Test
    fun matchesTheSdksSetEulerRadWhichIsTf2sSetRpy() {
        for (r in listOf(-0.3, 0.0, 0.25)) {
            for (p in listOf(-0.4, 0.0, 0.15)) {
                for (y in listOf(-3.0, 0.0, 1.1)) {
                    val sdk = com.segway.robot.algo.tf.Quaternion(0f, 0f, 0f, 0f)
                    sdk.setEulerRad(y.toFloat(), p.toFloat(), r.toFloat())
                    val q = HeadAttitude.quaternion(r, p, y)
                    assertArrayEquals(doubleArrayOf(sdk.x.toDouble(), sdk.y.toDouble(), sdk.z.toDouble(), sdk.w.toDouble()), q, 1e-6)
                }
            }
        }
    }

    @Test
    fun aUnitQuaternionThatReturnsAllThreeAngles() {
        for (r in listOf(-0.5, -0.02, 0.0, 0.1, 0.4)) {
            for (p in listOf(-1.0, -0.1, 0.0, 0.2, 1.2)) {
                for (y in listOf(-3.1, -1.0, 0.0, 0.5, 3.1)) {
                    val q = HeadAttitude.quaternion(r, p, y)
                    assertEquals(1.0, sqrt(q.sumOf { it * it }), 1e-12)
                    val (x, qy, z, w) = q.toList()
                    assertEquals(r, Math.atan2(2 * (w * x + qy * z), 1 - 2 * (x * x + qy * qy)), 1e-9)
                    assertEquals(p, Math.asin(2 * (w * qy - z * x)), 1e-9)
                    assertEquals(y, Math.atan2(2 * (w * z + x * qy), 1 - 2 * (qy * qy + z * z)), 1e-9)
                }
            }
        }
    }

    @Test
    fun nearLevelTheOrderBarelyChangesTheTilt() {
        // The order is assumed (Segway does not say). With pitch 10 and roll 5 degrees, the head's z
        // axis differs by under 0.1 degree between yaw-pitch-roll and the reverse order.
        val p = Math.toRadians(10.0)
        val r = Math.toRadians(5.0)
        val qx = doubleArrayOf(sin(r / 2), 0.0, 0.0, cos(r / 2))
        val qy = doubleArrayOf(0.0, sin(p / 2), 0.0, cos(p / 2))
        val a = rotate(HeadAttitude.quaternion(r, p, 0.0), doubleArrayOf(0.0, 0.0, 1.0))
        val b = rotate(mul(qx, qy), doubleArrayOf(0.0, 0.0, 1.0))
        val angle = Math.toDegrees(Math.acos(minOf(1.0, a[0] * b[0] + a[1] * b[1] + a[2] * b[2])))
        assertTrue("angle=$angle", angle < 0.1)
    }

    @Test
    fun theUnprovidedCovarianceFollowsSensorMsgsImu() {
        val c = HeadImuPublisher.notProvided()
        assertEquals(9, c.size)
        assertEquals(-1.0, c[0], 0.0)
        assertTrue(c.drop(1).all { it == 0.0 })
        // A fresh array each time: a message must not share it.
        assertTrue(c !== HeadImuPublisher.notProvided())
    }

    private fun assertArrayEquals(expected: DoubleArray, actual: DoubleArray, tol: Double) {
        assertEquals(expected.size, actual.size)
        // q and -q are the same rotation; the formulas here never flip the sign, so compare directly.
        for (i in expected.indices) assertTrue("[$i] ${expected[i]} vs ${actual[i]}", abs(expected[i] - actual[i]) <= tol)
    }

    /** The Hamilton product a * b of quaternions as x, y, z, w. */
    private fun mul(a: DoubleArray, b: DoubleArray): DoubleArray {
        val (ax, ay, az, aw) = a.toList()
        val (bx, by, bz, bw) = b.toList()
        return doubleArrayOf(
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )
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
