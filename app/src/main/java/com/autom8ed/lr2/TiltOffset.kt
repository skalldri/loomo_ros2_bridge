package com.autom8ed.lr2

/**
 * A constant error in the base's tilt, removed before the app publishes it (BridgeConfig
 * `tilt.roll_offset_deg` and `tilt.pitch_offset_deg`, both 0 by default).
 *
 * The base controller's BASE_IMU angles can carry a constant offset: on one Loomo, balancing on a
 * spot that a digital level read as level, the corrected roll (BaseTilt) read +0.89 degree, and on
 * a spot the level read as +1.4 degree (right side lower) it read +2.27. Segway's software applies
 * no calibration of its own on the Android side and exposes no stored zero, so the offset is
 * measured on the robot and set here. The offsets are what the app reads on a level floor, in
 * degrees with the ROS signs (roll positive right side down, pitch positive nose down); they are
 * subtracted, so a level base reads 0.
 *
 * Plain subtraction of the angles. For a roll offset it is exact when the error is a rotation of
 * the controller's sensor about the base's x axis: base_odom -> base_link is Ry(pitch) Rx(roll),
 * and Ry(p) Rx(r) Rx(-r0) = Ry(p) Rx(r - r0). For a pitch offset it is exact when the error is in
 * the level reference (Ry(-p0) Ry(p) Rx(r) = Ry(p - p0) Rx(r)). The other two cases differ in
 * second order: for a 1 degree offset, base_link's z axis points at most 0.03 degree away from the
 * exact correction within +-10 degrees of pitch and +-5 of roll (0.14 degree at +-20 of both).
 *
 * Only the base's own tilt is corrected: everything mounted on base_link follows through TF.
 * HEAD_WORLD_IMU (/loomo/head/world_imu, off by default) is left alone. It is not a head IMU: its
 * pitch is the base's pitch plus the head joint, so a base pitch offset hides in its -4 degree
 * constant, and its roll is a near-constant +0.2 degree that carries no base roll, so subtracting the
 * roll offset from it would add an error (measured on the robot, 2026-10-24).
 */
class TiltOffset(val rollDeg: Double = 0.0, val pitchDeg: Double = 0.0) {
    private val rollRad = Math.toRadians(rollDeg)
    private val pitchRad = Math.toRadians(pitchDeg)

    /** The base's roll in radians with the offset removed, from BaseTilt.rollRad's [measuredRad]. */
    fun roll(measuredRad: Double): Double = measuredRad - rollRad

    /** The base's pitch in radians with the offset removed, from BaseTilt.pitchRad's [measuredRad]. */
    fun pitch(measuredRad: Double): Double = measuredRad - pitchRad

    override fun toString() = "tilt.roll_offset_deg=$rollDeg tilt.pitch_offset_deg=$pitchDeg"

    companion object {
        /** The largest offset accepted from the configuration, in degrees either way. */
        const val MAX_DEG = 5.0
    }
}
