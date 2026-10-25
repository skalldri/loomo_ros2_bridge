package com.autom8ed.lr2

import kotlin.math.cos
import kotlin.math.sin

/**
 * The base's tilt in ROS terms (REP 103: x forward, y left, z up), from the SDK's BASE_IMU sample.
 *
 * Segway's locomotion service receives the base controller's pitch, roll and yaw as int16 counts
 * of 0.01 degree, and converts them to radians as `raw * PI / 32767`, as if full scale were 180
 * degrees. The true angle is `raw * PI / 18000`, so BASE_IMU's floatData holds 18000 / 32767 =
 * 0.549 times the true angles. [SDK_TO_TRUE] (32767 / 18000) undoes that exactly.
 *
 * floatData[0] is the pitch, with the sign of ROS pitch (nose down positive). floatData[1] is the
 * roll, with the opposite sign to ROS roll; the service passes it through unchanged, so it is
 * negated here. floatData[2] (yaw) has the same scale error but is not used: the heading comes
 * from Pose2D.
 *
 * Only BASE_IMU carries the base's tilt. Do not query Sensor.BODY_IMU on a Loomo: its native
 * library does not implement it, and the failure happens inside Segway's service.
 */
object BaseTilt {
    /** Undoes the service's `PI / 32767` per count; the counts are 0.01 degree (`PI / 18000`). */
    const val SDK_TO_TRUE: Double = 32767.0 / 18000.0

    /** Pitch about +y in radians, nose down positive, from BASE_IMU floatData[0]. */
    fun pitchRad(sdkPitch: Float): Double = sdkPitch.toDouble() * SDK_TO_TRUE

    /** Roll about +x in radians, left side up positive, from BASE_IMU floatData[1]. */
    fun rollRad(sdkRoll: Float): Double = -sdkRoll.toDouble() * SDK_TO_TRUE

    /**
     * The rotation base_odom -> base_link for [roll] and [pitch] (yaw zero), as x, y, z, w. The
     * same roll-pitch-yaw formula as tf2's setRPY and the SDK's Quaternion.setEulerRad.
     */
    fun quaternion(roll: Double, pitch: Double): DoubleArray {
        val cr = cos(roll * 0.5)
        val sr = sin(roll * 0.5)
        val cp = cos(pitch * 0.5)
        val sp = sin(pitch * 0.5)
        return doubleArrayOf(sr * cp, cr * sp, -sr * sp, cr * cp)
    }
}
