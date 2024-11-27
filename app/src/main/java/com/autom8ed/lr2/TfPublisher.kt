package com.autom8ed.lr2

import android.content.Context
import android.util.Log
import com.segway.robot.algo.Pose2D
import com.segway.robot.algo.tf.AlgoTfData
import com.segway.robot.algo.tf.AlgoTfRequest
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.Sensor
import com.segway.robot.sdk.perception.sensor.SensorData
import geometry_msgs.msg.Quaternion
import geometry_msgs.msg.Transform
import geometry_msgs.msg.TransformStamped
import geometry_msgs.msg.Vector3
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.qos.QoSProfile
import java.util.concurrent.BlockingQueue
import java.util.concurrent.LinkedBlockingDeque
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread
import kotlin.math.cos
import kotlin.math.sin


class TfPublisher(ctx: Context, node: RosNode, mSensorInterface: SensorInterface) {

    class TfNeededContext(
        timestamp: Long,
        baseImu: SensorData,
        headImu: SensorData,
        pose2D: SensorData
    ) {
        val mTimestamp: Long = timestamp
        val mBaseImu: SensorData = baseImu
        val mHeadImu: SensorData = headImu
        val mPose2D: SensorData = pose2D
    }

    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private var mTfPublisher: Publisher<tf2_msgs.msg.TFMessage>
    private var mOdometryPublisher: Publisher<nav_msgs.msg.Odometry>
    private var mJointStatePublisher: Publisher<sensor_msgs.msg.JointState>
    private val mNode: RosNode = node
    private var mThread: Thread? = null
    private val mThreadRun: AtomicBoolean = AtomicBoolean(true)
    private val mTimestampQueue: BlockingQueue<TfNeededContext> =
        LinkedBlockingDeque<TfNeededContext>()

    private val TAG: String = "TfPublisher"

    init {
        mBindSensorListener = object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindSensorListener onBind() called")
            }

            override fun onUnbind(reason: String) {
                Log.d(TAG, "mBindSensorListener onUnbind() called with: reason = [$reason]")
            }
        }
        mSensor.bindService(ctx, mBindSensorListener)
        mTfPublisher = mNode.node.createPublisher(tf2_msgs.msg.TFMessage::class.java, "/tf")
        mOdometryPublisher =
            mNode.node.createPublisher(nav_msgs.msg.Odometry::class.java, "/loomo/odom")
        mJointStatePublisher = mNode.node.createPublisher(
            sensor_msgs.msg.JointState::class.java,
            "/loomo/joint_states"
        )

        start()
    }

    fun indicateTfNeededAtTime(ctx: TfNeededContext) {
        mTimestampQueue.add(ctx)
    }

    fun start() {
        stop()

        mThreadRun.set(true)
        mThread = thread() {
            while (mThreadRun.get()) {
                val ctx: TfNeededContext = mTimestampQueue.take()
                publishTf(ctx)
            }
        }
    }

    fun stop() {
        // Check if a thread is already running
        // and restart if needed
        if (mThread != null) {
            // Stop the thread if it's running
            if (mThread!!.isAlive) {
                mThreadRun.set(false)
                mThread!!.join()
            }
        }
    }

    fun loomoFrameToRosFrame(frame: String): String {
        // Catch and translate some known frame IDs
        when (frame) {
            Sensor.BASE_POSE_FRAME -> return TfPublisherConstants.BASE_LINK
            Sensor.NECK_POSE_FRAME -> return TfPublisherConstants.NECK_LINK
            Sensor.RS_DEPTH_FRAME -> return TfPublisherConstants.REALSENSE_DEPTH_FRAME_ID
            Sensor.RS_COLOR_FRAME -> return TfPublisherConstants.REALSENSE_COLOR_FRAME_ID
            Sensor.RS_FE_FRAME -> return TfPublisherConstants.FISHEYE_FRAME_ID
            Sensor.BASE_ODOM_FRAME -> return TfPublisherConstants.BASE_ODOM
        }

        return frame
    }

    private fun algoTfDataToTransformStamped(tfData: AlgoTfData): TransformStamped {
        val vector3: Vector3 = Vector3()
        vector3.setX(tfData.t.x.toDouble())
        vector3.setY(tfData.t.y.toDouble())
        vector3.setZ(tfData.t.z.toDouble())
        val quaternion: Quaternion = Quaternion()
        quaternion.setX(tfData.q.x.toDouble())
        quaternion.setY(tfData.q.y.toDouble())
        quaternion.setZ(tfData.q.z.toDouble())
        quaternion.setW(tfData.q.w.toDouble())
        val transform: Transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)
        val transformStamped: TransformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(loomoFrameToRosFrame(tfData.tgtFrameID))
        transformStamped.header.setFrameId(loomoFrameToRosFrame(tfData.srcFrameID))

        // These values are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(tfData.timeStamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (tfData.timeStamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun getLoomoToOpticalFrameTf(
        tgtFrame: String,
        srcFrame: String,
        timestamp: Long
    ): TransformStamped {
        val vector3 = Vector3()

        // Assume same physical position
        vector3.setX(0.0)
        vector3.setY(0.0)
        vector3.setZ(0.0)

        // ROS expects OpenGL notation (+Z into the scene, +Y top -> bottom, +X left -> right),
        // But Loomo TF assumes +Z aligned with gravity.
        // Apply -90 degree rotation around the X axis, and +90 degree rotation around the Y axis to re-align
        val quaternion = Quaternion()
        quaternion.setX(-0.5)
        quaternion.setY(0.5)
        quaternion.setZ(-0.5)
        quaternion.setW(0.5)

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(tgtFrame) //TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID)
        transformStamped.header.setFrameId(srcFrame) // TfPublisherConstants.REALSENSE_DEPTH_FRAME_ID)

        // These values are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun getUltrasonicTransform(timestamp: Long): TransformStamped {
        val vector3 = Vector3()

        // Measured with a ruler...
        // TODO: get way way more accurate measurements
        vector3.setX(TfPublisherConstants.ULTRASONIC_FORWARD_M);
        vector3.setY(0.0);
        vector3.setZ(TfPublisherConstants.ULTRASONIC_HEIGHT_M);

        // No rotation component: identity quaternion
        val quaternion = Quaternion()
        quaternion.setX(0.0)
        quaternion.setY(0.0)
        quaternion.setZ(0.0)
        quaternion.setW(1.0)

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.ULTRASONIC_FRAME_ID)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_LINK)

        // These values are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun getIrLeftTransform(timestamp: Long): TransformStamped {
        val vector3 = Vector3()

        // Measured with calipers relative to the USS
        vector3.setX(TfPublisherConstants.ULTRASONIC_FORWARD_M) // IR sensors are very close to USS
        vector3.setY(0.0 + TfPublisherConstants.IR_CENTER_OFFSET_M)
        vector3.setZ(TfPublisherConstants.ULTRASONIC_HEIGHT_M + TfPublisherConstants.IR_HEIGHT_OFFSET_FROM_USS_M)

        // Looks ~ +10 degree rotation
        val quaternion = Quaternion()
        quaternion.setX(0.0)
        quaternion.setY(0.0)
        quaternion.setZ(0.087)
        quaternion.setW(0.996)

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.IR_LEFT_FRAME_ID)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_LINK)

        // These values are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun getIrRightTransform(timestamp: Long): TransformStamped {
        val vector3 = Vector3()

        // Measured with calipers relative to the USS
        vector3.setX(TfPublisherConstants.ULTRASONIC_FORWARD_M) // IR sensors are very close to USS
        vector3.setY(0.0 - TfPublisherConstants.IR_CENTER_OFFSET_M)
        vector3.setZ(TfPublisherConstants.ULTRASONIC_HEIGHT_M + TfPublisherConstants.IR_HEIGHT_OFFSET_FROM_USS_M)

        // Looks ~ -10 degree rotation
        val quaternion = Quaternion()
        quaternion.setX(0.0)
        quaternion.setY(0.0)
        quaternion.setZ(-0.087)
        quaternion.setW(0.996)

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.IR_RIGHT_FRAME_ID)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_LINK)

        // These values are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun toQuaternion(
        roll: Double,
        pitch: Double,
        yaw: Double
    ): Quaternion // roll (x), pitch (y), yaw (z), angles are in radians
    {
        // Abbreviations for the various angular functions
        val q = Quaternion()

        val cr: Double = cos(roll * 0.5);
        val sr: Double = sin(roll * 0.5);
        val cp: Double = cos(pitch * 0.5);
        val sp: Double = sin(pitch * 0.5);
        val cy: Double = cos(yaw * 0.5);
        val sy: Double = sin(yaw * 0.5);

        q.w = cr * cp * cy + sr * sp * sy;
        q.x = sr * cp * cy - cr * sp * sy;
        q.y = cr * sp * cy + sr * cp * sy;
        q.z = cr * cp * sy - sr * sp * cy;

        return q;
    }

    private fun getBaseOdomTransform(baseImuData: SensorData): TransformStamped {
        //Log.i(TAG, "Base IMU Sensor: ${mBaseImuData.floatData[0]}, ${mBaseImuData.floatData[1]}, ${mBaseImuData.floatData[2]}")

        //val mWorldImuData =
        //    mSensor.querySensorData(listOf(Sensor.HEAD_WORLD_IMU))[0]
        //Log.i(TAG, "World IMU Sensor: ${mWorldImuData.floatData[0]}, ${mWorldImuData.floatData[1]}, ${mWorldImuData.floatData[2]}")

        // In Loomo World:
        //
        // Pitch == vertical tilt
        // Roll == Side-to-side tilt ????
        // Yaw == In-place Rotation ???
        //
        // val mBasePitch: Float = mBaseImu.getFloatData().get(0)
        // val mBaseRoll: Float = mBaseImu.getFloatData().get(1)
        // val mBaseYaw: Float = mBaseImu.getFloatData().get(2)

        // In ROS world:
        // +X == Robot forward
        // +Y == Robot Left
        // +Z == Robot Up

        // So, pitch -> rotation around the Y axis

        val vector3 = Vector3()

        vector3.setX(0.0)
        vector3.setY(0.0)
        vector3.setZ(0.0)

        // Use an SDK quaternion to construct a ROS quaternion
        var q: com.segway.robot.algo.tf.Quaternion =
            com.segway.robot.algo.tf.Quaternion(0F, 0F, 0F, 0F)
        q.setEulerRad(0.0F, baseImuData.floatData[0], baseImuData.floatData[1])

        val quaternion = Quaternion()
        quaternion.setX(q.x.toDouble())
        quaternion.setY(q.y.toDouble())
        quaternion.setZ(q.z.toDouble())
        quaternion.setW(q.w.toDouble())

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.BASE_LINK)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_ODOM)

        // These timestamps are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(baseImuData.timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec =
            (baseImuData.timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    private fun getLoomoOdomTransform(pose2D: Pose2D): TransformStamped {
        //Log.i(TAG, "Base IMU Sensor: ${mBaseImuData.floatData[0]}, ${mBaseImuData.floatData[1]}, ${mBaseImuData.floatData[2]}")

        //val mWorldImuData =
        //    mSensor.querySensorData(listOf(Sensor.HEAD_WORLD_IMU))[0]
        //Log.i(TAG, "World IMU Sensor: ${mWorldImuData.floatData[0]}, ${mWorldImuData.floatData[1]}, ${mWorldImuData.floatData[2]}")

        // In Loomo World:
        //
        // Pitch == vertical tilt
        // Roll == Side-to-side tilt ????
        // Yaw == In-place Rotation ???
        //
        // val mBasePitch: Float = mBaseImu.getFloatData().get(0)
        // val mBaseRoll: Float = mBaseImu.getFloatData().get(1)
        // val mBaseYaw: Float = mBaseImu.getFloatData().get(2)

        // In ROS world:
        // +X == Robot forward
        // +Y == Robot Left
        // +Z == Robot Up

        // So, pitch -> rotation around the Y axis

        val vector3 = Vector3()

        vector3.setX(pose2D.x.toDouble())
        vector3.setY(pose2D.y.toDouble())
        vector3.setZ(0.0)

        // Use an SDK quaternion to construct a ROS quaternion
        var q: com.segway.robot.algo.tf.Quaternion =
            com.segway.robot.algo.tf.Quaternion(0F, 0F, 0F, 0F)
        q.setEulerRad(pose2D.theta, 0F, 0F)

        val quaternion = Quaternion()
        quaternion.setX(q.x.toDouble())
        quaternion.setY(q.y.toDouble())
        quaternion.setZ(q.z.toDouble())
        quaternion.setW(q.w.toDouble())

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.BASE_ODOM)
        transformStamped.header.setFrameId(TfPublisherConstants.LOOMO_ODOM)

        // These timestamps are in microseconds
        transformStamped.header.stamp.sec =
            TimeUnit.SECONDS.convert(pose2D.timestamp, TimeUnit.MICROSECONDS).toInt();
        transformStamped.header.stamp.nanosec = (pose2D.timestamp % (1000 * 1000)).toInt() * (1000);

        return transformStamped
    }

    // Capture context for an event that occurred at time T
    fun captureTfContext(timestamp: Long): TfNeededContext {
        val sensorData =
            mSensor.querySensorData(listOf(Sensor.POSE_2D, Sensor.BASE_IMU, Sensor.HEAD_WORLD_IMU))
        val pose2DData = sensorData[0]
        val baseImuData = sensorData[1]
        val headImuData = sensorData[2]

        // Log.i(TAG, "Base IMU Data: ${baseImuData.floatData[0]},${baseImuData.floatData[1]},${baseImuData.floatData[2]}")
        // Log.i(TAG, "Head IMU Data: ${headImuData.floatData[0]},${headImuData.floatData[1]},${headImuData.floatData[2]}")

        val ctx: TfNeededContext = TfNeededContext(timestamp, baseImuData, headImuData, pose2DData)
        return ctx
    }

    private fun publishTf(ctx: TfNeededContext) {
        // Bail out if we haven't bound to the sensor service yet
        if (!mSensor.isBind) {
            return
        }

        // Construct a list of transforms that can be queried directly from Loomo's SDK

        // TODO: calculate initial arraylist capacity based on expected robot transforms
        val transforms: MutableList<geometry_msgs.msg.TransformStamped> =
            MutableList(0) { geometry_msgs.msg.TransformStamped() }

        // base_link -> neck_link
        // neck_link is in the center of the neck, without any rotation component
        val T_BaseLink_NeckLink =
            AlgoTfRequest(
                Sensor.HEAD_POSE_Y_FRAME,
                Sensor.BASE_POSE_FRAME,
                ctx.mTimestamp,
                100 /* lookup threshold */
            )

        // neck_link -> neck_yaw_link
        // neck_yaw_link has the same location as neck_link, but it incorporates the rotation of the neck
        //val T_NeckLink_NeckYawLink =
        //    AlgoTfRequest(Sensor.HEAD_POSE_Y_FRAME, Sensor.NECK_POSE_FRAME, timestamp, 100 /* lookup threshold */)

        // neck_yaw_link -> realsense_depth
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_RealsenseDepth =
            AlgoTfRequest(
                Sensor.RS_DEPTH_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                ctx.mTimestamp,
                100 /* lookup threshold */
            )

        // neck_yaw_link -> realsense_color
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_RealsenseColor =
            AlgoTfRequest(
                Sensor.RS_COLOR_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                ctx.mTimestamp,
                100 /* lookup threshold */
            )

        // neck_yaw_link -> realsense_depth
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_Fisheye =
            AlgoTfRequest(
                Sensor.RS_FE_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                ctx.mTimestamp,
                100 /* lookup threshold */
            )

        val requests: List<AlgoTfRequest> = listOf(
            T_BaseLink_NeckLink,
            T_NeckYawLink_RealsenseDepth,
            T_NeckYawLink_RealsenseColor,
            T_NeckYawLink_Fisheye
        )
        val results = mSensor.getMassiveTfData(requests)

        for (r in results) {
            //val tfData = mSensor.getTfData(req.tgtFrameID, req.srcFrameID, req.timeStamp, 100)

            if (r.err_code == 0) {
                transforms.add(algoTfDataToTransformStamped(r))
            }
        }

        val pose2D: Pose2D = mSensor.sensorDataToPose2D(ctx.mPose2D)

        if (transforms.size != 0) {
            // Add static / fixup transforms
            transforms.add(
                getLoomoToOpticalFrameTf(
                    TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID,
                    TfPublisherConstants.REALSENSE_DEPTH_FRAME_ID,
                    T_NeckYawLink_RealsenseDepth.timeStamp
                )
            )
            transforms.add(
                getLoomoToOpticalFrameTf(
                    TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID,
                    TfPublisherConstants.REALSENSE_COLOR_FRAME_ID,
                    T_NeckYawLink_RealsenseColor.timeStamp
                )
            )
            transforms.add(
                getLoomoToOpticalFrameTf(
                    TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID,
                    TfPublisherConstants.FISHEYE_FRAME_ID,
                    T_NeckYawLink_Fisheye.timeStamp
                )
            )
            transforms.add(getUltrasonicTransform(ctx.mBaseImu.timestamp))
            transforms.add(getIrLeftTransform(ctx.mBaseImu.timestamp))
            transforms.add(getIrRightTransform(ctx.mBaseImu.timestamp))

            // Add calculated platform tilt
            // TODO: there's a significant delay between when a camera frame
            // is captured and when we capture this IMU data. Consider creating a dedicated IMU capture
            // thread, and interpolating between samples to get a better estimate of the platform tilt
            // at the time the camera captured
            transforms.add(getBaseOdomTransform(ctx.mBaseImu))

            transforms.add(getLoomoOdomTransform(pose2D))

            // Fire off the completed transform message
            val tfMsg: tf2_msgs.msg.TFMessage = tf2_msgs.msg.TFMessage()
            tfMsg.transforms = transforms
            mTfPublisher.publish(tfMsg)
        }

        publishOdometry(pose2D)

        publishJointState(ctx.mHeadImu, results[0])
    }

    private fun publishOdometry(pose2D: Pose2D) {

        val odomMsg: nav_msgs.msg.Odometry = nav_msgs.msg.Odometry()
        odomMsg.header.frameId = TfPublisherConstants.LOOMO_ODOM
        odomMsg.childFrameId = TfPublisherConstants.BASE_ODOM

        odomMsg.pose.pose.position.x = pose2D.x.toDouble()
        odomMsg.pose.pose.position.y = pose2D.y.toDouble()

        val q: com.segway.robot.algo.tf.Quaternion =
            com.segway.robot.algo.tf.Quaternion(0F, 0F, 0F, 0F)
        q.setEulerRad(pose2D.theta, 0.0F, 0.0F)

        odomMsg.pose.pose.orientation.x = q.x.toDouble()
        odomMsg.pose.pose.orientation.y = q.y.toDouble()
        odomMsg.pose.pose.orientation.z = q.z.toDouble()
        odomMsg.pose.pose.orientation.w = q.w.toDouble()

        // Intentionally not setting the mOdomMsg.pose.covariance matrix

        odomMsg.twist.twist.linear.x = pose2D.linearVelocity.toDouble()
        odomMsg.twist.twist.angular.z = pose2D.linearVelocity.toDouble()

        // Intentionally not setting the mOdomMsg.twist.covariance matrix

        // These timestamps are in microseconds
        odomMsg.header.stamp.sec =
            TimeUnit.SECONDS.convert(pose2D.timestamp, TimeUnit.MICROSECONDS).toInt();
        odomMsg.header.stamp.nanosec = (pose2D.timestamp % (1000 * 1000)).toInt() * (1000);

        mOdometryPublisher.publish(odomMsg)
    }

    private fun publishJointState(headImuData: SensorData, T_BaseLink_NeckLink: AlgoTfData) {
        val jointStateMsg: sensor_msgs.msg.JointState = sensor_msgs.msg.JointState()

        var positions: DoubleArray = DoubleArray(2)

        // I don't think this comes from real IMU data, I think it's just the
        // joint state being published as a "fake" IMU
        // TODO: validate this assumption
        positions[0] =
            T_BaseLink_NeckLink.q.rollRad.toDouble() // Head "Yaw". Segway calls rotation around the Z-axis "Roll"
        positions[1] =
            (-1.0) * headImuData.floatData[0].toDouble() // Head Pitch. Segway roll info is backwards compared to what ROS expects

        jointStateMsg.name = listOf("neck_yaw_joint", "head_pitch_joint")
        jointStateMsg.position = positions

        // My current converter code barfs if these arrays are not initialized to a real value
        jointStateMsg.velocity = DoubleArray(2) { 0.0 }
        jointStateMsg.effort = DoubleArray(2) { 0.0 }

        // These timestamps are in microseconds
        jointStateMsg.header.stamp.sec =
            TimeUnit.SECONDS.convert(headImuData.timestamp, TimeUnit.MICROSECONDS).toInt();
        jointStateMsg.header.stamp.nanosec =
            (headImuData.timestamp % (1000 * 1000)).toInt() * (1000);

        mJointStatePublisher.publish(jointStateMsg)
    }
}