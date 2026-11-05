package com.autom8ed.lr2

import kotlin.math.cos
import kotlin.math.sin

/**
 * The head's attitude in the world and its roll joint in ROS terms (REP 103: x forward, y left,
 * z up), from the SDK's HEAD_WORLD_IMU and HEAD_JOINT_ROLL samples.
 *
 * HEAD_WORLD_IMU's floatData is [pitch, roll, yaw]. Despite its name it is not a measurement from an
 * IMU in the head (measured on the robot, 2026-10-24, against the base tilt, the head joints, a
 * LIDAR IMU on the column, the fisheye image and a digital level):
 *  - pitch = the base's pitch + the head pitch joint - 4.0 deg (gain 0.99-1.00, residual 0.1-0.2
 *    deg parked), so it adds nothing to base_odom -> base_link and head_pitch_joint;
 *  - roll is a near-constant +0.2 deg: it ignored a base roll of up to 9 deg and a 14 deg sway
 *    of the body that the head's own fisheye camera saw;
 *  - yaw is always 0, through hundreds of turns.
 * It is presumably computed by Segway's firmware from the base's pitch and the head encoder
 * (inferred). Hence BridgeConfig head.imu defaults to off. HEAD_JOINT_ROLL's floatData[0] is the
 * head's roll joint; it is always 0 (the head has no roll motor).
 *
 * Scale: Segway's locomotion service converts each of the head controller's int16 angles with
 * `raw / 10427.832` (182 counts per degree, so 32767 counts are 180.04 degrees: pi / 32767 rad per
 * count to within 0.02%). That is how the head encodes its angles, so no correction is applied.
 * BASE_IMU, from the base controller, does need one (see BaseTilt).
 *
 * Signs. ROS: roll about +x, left side up positive; pitch about +y, nose down positive; yaw about
 * +z, counterclockwise seen from above positive.
 *  - Pitch: `-floatData[0]`. The service passes the head's world pitch through unchanged, and it is
 *    positive nose up. Evidence, all indirect:
 *    1. HEAD_JOINT_PITCH goes through the same conversion. Parked on 2026-10-14 it read -0.111 rad
 *       while Segway's TF gave +0.112 rad (nose down) for the same joint (see TfPublisher).
 *    2. In a recording from 2026-10-13, floatData[0] moved against BASE_IMU's pitch, which has the
 *       ROS sign: fitted gain -0.37 to -0.60 depending on the lag. The head partly follows the
 *       body's lean, so the ROS head pitch must move with it.
 *    3. The app's first version published `-floatData[0]` as the head pitch joint.
 *  - Yaw: `+floatData[2]`. The service negates the raw world yaw, exactly as it negates the raw
 *    joint yaw, and the joint yaw's sign was checked against the base's turns (the head
 *    counter-rotates). Inferred from that; not measured for the world yaw. Its zero is arbitrary
 *    (the head IMU integrates it).
 *  - Roll: `+floatData[1]`. A guess, not measured: no recording has it yet. The raw pitch is
 *    positive nose up and the raw yaw positive clockwise seen from above. Those are rotations about
 *    "right" and "down", and the right-handed set they belong to has "forward" as its third axis.
 *    About forward, positive is right side down, which is also ROS's positive roll. The service
 *    passes the roll through unchanged. The base's roll, from a different board, has the opposite
 *    sign to ROS, so this one must be checked on the robot.
 *  - Joint roll: `+floatData[0]`, the same convention as the world roll. Also a guess. If the head
 *    has no roll motor it stays at zero and the sign does not matter.
 *
 * Order: the three angles are taken as yaw, then pitch, then roll (tf2's setRPY, the usual
 * convention for an attitude in the world). Assumed: Segway does not say. Near level the order
 * hardly matters for the tilt: with the head pitched 10 degrees and rolled 5 degrees, the direction
 * of the head's z axis differs by 0.08 degree between this order and the reverse one.
 */
object HeadAttitude {
    /** Pitch about +y in radians, nose down positive, from HEAD_WORLD_IMU floatData[0]. */
    fun pitchRad(sdkPitch: Float): Double = -sdkPitch.toDouble()

    /** Roll about +x in radians, left side up positive, from HEAD_WORLD_IMU floatData[1]. */
    fun rollRad(sdkRoll: Float): Double = sdkRoll.toDouble()

    /** Yaw about +z in radians, counterclockwise positive, from HEAD_WORLD_IMU floatData[2]. */
    fun yawRad(sdkYaw: Float): Double = sdkYaw.toDouble()

    /** The roll joint about +x in radians, left side up positive, from HEAD_JOINT_ROLL floatData[0]. */
    fun jointRollRad(sdkJointRoll: Float): Double = sdkJointRoll.toDouble()

    /** The head's orientation in the world, as x, y, z, w, from HEAD_WORLD_IMU's floatData. */
    fun orientation(floatData: FloatArray): DoubleArray =
        quaternion(rollRad(floatData[1]), pitchRad(floatData[0]), yawRad(floatData[2]))

    /**
     * The rotation for [roll], [pitch] and [yaw] (radians) as x, y, z, w: tf2's setRPY, which is
     * yaw about z, then pitch about the new y, then roll about the new x.
     */
    fun quaternion(roll: Double, pitch: Double, yaw: Double): DoubleArray {
        val cr = cos(roll * 0.5)
        val sr = sin(roll * 0.5)
        val cp = cos(pitch * 0.5)
        val sp = sin(pitch * 0.5)
        val cy = cos(yaw * 0.5)
        val sy = sin(yaw * 0.5)
        return doubleArrayOf(
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )
    }
}
