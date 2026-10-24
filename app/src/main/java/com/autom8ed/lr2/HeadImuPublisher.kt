package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.perception.sensor.Sensor
import com.segway.robot.sdk.perception.sensor.SensorData
import org.ros2.rcljava.publisher.Publisher

/**
 * Publishes what the head's own sensors report (BridgeConfig head.imu, default on). It only
 * reads: nothing here commands the head.
 *  - [TOPIC_WORLD_IMU] (sensor_msgs/Imu): HEAD_WORLD_IMU, the head's attitude in the world, as an
 *    orientation only, in the URDF's head frame ([FRAME_ID]). The SDK gives no angular velocity or
 *    acceleration for the head, so both are marked as not provided (covariance[0] = -1, as
 *    sensor_msgs/Imu specifies). The orientation's covariance is unknown (all zeros).
 *  - [TOPIC_JOINT_ROLL] (sensor_msgs/JointState, one joint, [JOINT_ROLL_NAME]): HEAD_JOINT_ROLL.
 *    The URDF has no head roll joint, so it is not in /loomo/joint_states. A JointState rather than
 *    a bare std_msgs/Float64, so the sample keeps its stamp.
 * The conversions and signs are in [HeadAttitude].
 *
 * Called on TfPublisher's 20 Hz timer, on its thread (the SDK is only queried from there), with a
 * query of its own, so a failure here cannot stop the odometry. Each sample is published once
 * ([SampleGate]), stamped with its own SDK timestamp on the Jetson's clock ([TimeSync]).
 * HEAD_WORLD_IMU's stamp is its pitch's: the service reads pitch, roll and yaw one after the other
 * and keeps only the first stamp.
 */
class HeadImuPublisher(node: RosNode) {
    private val mImuPublisher: Publisher<sensor_msgs.msg.Imu> =
        node.node.createPublisher(sensor_msgs.msg.Imu::class.java, TOPIC_WORLD_IMU)
    private val mJointRollPublisher: Publisher<sensor_msgs.msg.JointState> =
        node.node.createPublisher(sensor_msgs.msg.JointState::class.java, TOPIC_JOINT_ROLL)

    // Only touched on the tf-publisher thread; the counters are read by the stats line.
    private val mWorldGate = SampleGate()
    private val mJointRollGate = SampleGate()
    @Volatile private var mFailures = 0L

    /** One part of the StreamStats line: new and repeated samples of each source, failed queries. */
    fun statsLine(): String =
        "head_imu new=${mWorldGate.passed} repeat=${mWorldGate.repeats}" +
            " head_roll new=${mJointRollGate.passed} repeat=${mJointRollGate.repeats}" +
            " head_fail=$mFailures"

    /** Reads HEAD_WORLD_IMU and HEAD_JOINT_ROLL and publishes each new sample. Never throws. */
    fun publish(sensor: Sensor) {
        if (!sensor.isBind) {
            return
        }
        // No Loomo-clock stamps on the wire: wait for the first clock sample (TimeSync).
        if (!TimeSync.isReady()) {
            TimeSync.dropUnsynced()
            return
        }
        try {
            // Never add Sensor.BODY_IMU here: the Loomo does not implement it (see BaseTilt).
            val data = sensor.querySensorData(QUERY)
            val world = data[0]
            val jointRoll = data[1]
            if (mWorldGate.isNew(world.timestamp)) {
                publishWorld(world)
            }
            if (mJointRollGate.isNew(jointRoll.timestamp)) {
                publishJointRoll(jointRoll)
            }
        } catch (t: Throwable) {
            // At 20 Hz a persistent failure would flood logcat: log the first, then once a minute.
            val n = ++mFailures
            if (n == 1L || n % 1200 == 0L) {
                Log.e(TAG, "HEAD_WORLD_IMU/HEAD_JOINT_ROLL query or publish failed ($n so far)", t)
            }
        }
    }

    private fun publishWorld(world: SensorData) {
        val q = HeadAttitude.orientation(world.floatData)
        val msg = sensor_msgs.msg.Imu()
        msg.header.frameId = FRAME_ID
        TimeSync.toJetson(msg.header.stamp, world.timestamp, TimeSync.HEAD_IMU)
        msg.orientation.x = q[0]
        msg.orientation.y = q[1]
        msg.orientation.z = q[2]
        msg.orientation.w = q[3]
        // Orientation covariance stays all zeros: unknown. The two vectors stay zero, flagged as
        // not provided.
        msg.angularVelocityCovariance = notProvided()
        msg.linearAccelerationCovariance = notProvided()
        mImuPublisher.publish(msg)
    }

    private fun publishJointRoll(jointRoll: SensorData) {
        val msg = sensor_msgs.msg.JointState()
        TimeSync.toJetson(msg.header.stamp, jointRoll.timestamp, TimeSync.HEAD_JOINT_ROLL)
        msg.name = listOf(JOINT_ROLL_NAME)
        msg.position = doubleArrayOf(HeadAttitude.jointRollRad(jointRoll.floatData[0]))
        // Same as /loomo/joint_states: the converter needs real arrays here.
        msg.velocity = DoubleArray(1)
        msg.effort = DoubleArray(1)
        mJointRollPublisher.publish(msg)
    }

    companion object {
        private const val TAG = "HeadImuPublisher"
        const val TOPIC_WORLD_IMU = "/loomo/head/world_imu"
        const val TOPIC_JOINT_ROLL = "/loomo/head/joint_roll"
        /** The URDF's head link, the part that pitches (assumed to carry the head's IMU). */
        const val FRAME_ID = "head_pitch_link"
        const val JOINT_ROLL_NAME = "head_roll_joint"
        private val QUERY = listOf(Sensor.HEAD_WORLD_IMU, Sensor.HEAD_JOINT_ROLL)

        /** sensor_msgs/Imu's "no estimate" covariance: element 0 is -1. */
        fun notProvided(): DoubleArray = DoubleArray(9).also { it[0] = -1.0 }
    }
}
