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
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicLong
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread
import kotlin.math.cos
import kotlin.math.roundToLong
import kotlin.math.sin


/**
 * Publishes the robot's TF, odometry and joint states, all from one thread ("tf-publisher"):
 *  - per depth frame (30 Hz), queued by [indicateTfNeededAtTime]: the head and camera transforms
 *    and the joint states, looked up at the frame's time;
 *  - on its own 20 Hz timer, whatever the cameras do: Pose2D (/loomo/odom and, with tf.odom,
 *    loomo_odom -> base_odom) and the base IMU's tilt (base_odom -> base_link and the range
 *    sensors' mounts). These used to be captured per depth frame, so a stalled depth stream
 *    stopped the odometry and the tilt too. With head.imu, the same timer also publishes the
 *    head's attitude in the world and its roll joint (HeadImuPublisher).
 */
class TfPublisher(ctx: Context, node: RosNode, mSensorInterface: SensorInterface, config: BridgeConfig) {

    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private var mTfPublisher: Publisher<tf2_msgs.msg.TFMessage>
    private var mOdometryPublisher: Publisher<nav_msgs.msg.Odometry>
    private var mJointStatePublisher: Publisher<sensor_msgs.msg.JointState>
    private val mNode: RosNode = node
    private var mThread: Thread? = null
    private val mThreadRun: AtomicBoolean = AtomicBoolean(true)
    // Timestamps of frames for which a TF/odometry capture was requested. Bounded: the camera
    // callback offers and never blocks; the sensor queries run on this class's thread.
    private val mTimestampQueue: BlockingQueue<Long> = ArrayBlockingQueue<Long>(64)
    private val mQueueDrops = AtomicLong()

    // Pose2D and the base IMU are 20 Hz snapshots, polled on their own timer: each sample is
    // published once (see SampleGate). Only touched on the tf-publisher thread.
    private val mPose2DGate = SampleGate()
    private val mBaseImuGate = SampleGate()
    @Volatile private var mBaseTicks = 0L
    @Volatile private var mBaseTicksSkipped = 0L
    // Twist from Pose2D differences (BridgeConfig odom.twist=pose_diff), or null for the SDK's.
    private val mPoseDiffTwist: PoseDiffTwist? =
        if (config.odomTwistPoseDiff) PoseDiffTwist(windowUs = (config.odomTwistWindowS * 1e6).roundToLong()) else null
    // loomo_odom -> base_odom on /tf (BridgeConfig tf.odom, off by default: an odometry filter on
    // the ROS side owns base_odom's parent).
    private val mPublishOdomTf: Boolean = config.tfOdom
    // The head's own attitude and roll joint (BridgeConfig head.imu), or null when off.
    private val mHeadImu: HeadImuPublisher? = if (config.headImu) HeadImuPublisher(node) else null
    // The base tilt's constant error (BridgeConfig tilt.roll_offset_deg, tilt.pitch_offset_deg).
    private val mTiltOffset: TiltOffset = config.tiltOffset
    @Volatile private var mTwistFallbacks = 0L
    @Volatile private var mJointStateSkips = 0L

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

    /** Depth of the pending TF-capture queue and the number of dropped requests, for reporting. */
    fun queueDepth(): Int = mTimestampQueue.size
    fun queueDrops(): Long = mQueueDrops.get()

    /** One line for StatsReporter: capture queue, published and repeated SDK samples, fallbacks. */
    fun statsLine(): String =
        "tf queue depth=${queueDepth()} drops=${queueDrops()}" +
            " pose2d new=${mPose2DGate.passed} repeat=${mPose2DGate.repeats}" +
            " base_imu new=${mBaseImuGate.passed} repeat=${mBaseImuGate.repeats}" +
            " twist=${if (mPoseDiffTwist != null) "pose_diff" else "sdk"} sdk_fallbacks=$mTwistFallbacks" +
            " odom_tf=$mPublishOdomTf" +
            " base_ticks=$mBaseTicks skipped=$mBaseTicksSkipped" +
            " joint_state_skips=$mJointStateSkips" +
            " " + (mHeadImu?.statsLine() ?: "head_imu=off")

    /** Requests a TF/odometry capture for a frame taken at [timestampUs]. Never blocks. */
    fun indicateTfNeededAtTime(timestampUs: Long) {
        if (!mTimestampQueue.offer(timestampUs)) {
            val n = mQueueDrops.incrementAndGet()
            Log.e(TAG, "TF capture request dropped (queue full, ${mTimestampQueue.size}); total drops $n, stamp_us=$timestampUs")
        }
    }

    fun start() {
        stop()

        mThreadRun.set(true)
        mThread = thread(name = "tf-publisher") {
            // The base timer and the frame queue share this thread, so the SDK is only ever
            // queried from here. A frame waits at most one base publish, and the base timer runs
            // whether or not frames arrive.
            val baseTimer = FixedRate(BASE_PERIOD_NS, System.nanoTime())
            while (mThreadRun.get()) {
                val timestampUs: Long? = try {
                    mTimestampQueue.poll(baseTimer.waitNs(System.nanoTime()), TimeUnit.NANOSECONDS)
                } catch (e: InterruptedException) {
                    return@thread
                }
                if (timestampUs != null) {
                    try {
                        publishFrameTf(timestampUs)
                    } catch (t: Throwable) {
                        Log.e(TAG, "TF capture/publish failed for stamp_us=$timestampUs", t)
                    }
                }
                if (baseTimer.due(System.nanoTime())) {
                    mBaseTicks++
                    mBaseTicksSkipped = baseTimer.skipped
                    try {
                        publishBase()
                    } catch (t: Throwable) {
                        Log.e(TAG, "Pose2D/base IMU capture/publish failed", t)
                    }
                    // After the base, in a query of its own: it catches and counts its failures.
                    mHeadImu?.publish(mSensor)
                }
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

        TimeSync.toJetson(transformStamped.header.stamp, tfData.timeStamp, TimeSync.TF_ALGO)

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

        TimeSync.toJetson(transformStamped.header.stamp, timestamp)

        return transformStamped
    }

    private fun getUltrasonicTransform(stamp: builtin_interfaces.msg.Time): TransformStamped {
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

        copyStamp(transformStamped.header.stamp, stamp)

        return transformStamped
    }

    private fun getIrLeftTransform(stamp: builtin_interfaces.msg.Time): TransformStamped {
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

        copyStamp(transformStamped.header.stamp, stamp)

        return transformStamped
    }

    private fun getIrRightTransform(stamp: builtin_interfaces.msg.Time): TransformStamped {
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

        copyStamp(transformStamped.header.stamp, stamp)

        return transformStamped
    }

    private fun copyStamp(dst: builtin_interfaces.msg.Time, src: builtin_interfaces.msg.Time) {
        dst.sec = src.sec
        dst.nanosec = src.nanosec
    }

    /** [loomoUs] on the Jetson's clock (TimeSync), as a new stamp to copy into each message. */
    private fun jetsonStamp(loomoUs: Long, age: TimeSync.StampAge): builtin_interfaces.msg.Time {
        val stamp = builtin_interfaces.msg.Time()
        TimeSync.toJetson(stamp, loomoUs, age)
        return stamp
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

    private fun getBaseOdomTransform(baseImuData: SensorData, stamp: builtin_interfaces.msg.Time): TransformStamped {
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

        // BASE_IMU's angles are 0.549 times the true ones, and its roll has the opposite sign to
        // ROS roll: Segway's service scales the controller's counts wrongly (see BaseTilt). The
        // configured constant offset comes off after that (see TiltOffset).
        val pitch = mTiltOffset.pitch(BaseTilt.pitchRad(baseImuData.floatData[0]))
        val roll = mTiltOffset.roll(BaseTilt.rollRad(baseImuData.floatData[1]))
        val q = BaseTilt.quaternion(roll, pitch)

        val quaternion = Quaternion()
        quaternion.setX(q[0])
        quaternion.setY(q[1])
        quaternion.setZ(q[2])
        quaternion.setW(q[3])

        val transform = Transform()
        transform.setTranslation(vector3)
        transform.setRotation(quaternion)

        val transformStamped = TransformStamped()
        transformStamped.setTransform(transform)
        transformStamped.setChildFrameId(TfPublisherConstants.BASE_LINK)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_ODOM)

        copyStamp(transformStamped.header.stamp, stamp)

        return transformStamped
    }

    private fun getLoomoOdomTransform(pose2D: Pose2D, stamp: builtin_interfaces.msg.Time): TransformStamped {
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

        copyStamp(transformStamped.header.stamp, stamp)

        return transformStamped
    }

    /**
     * Pose2D and the base IMU, on the 20 Hz timer: base_odom -> base_link and the range sensors'
     * mounts for a new IMU sample, loomo_odom -> base_odom (only with tf.odom=true) and /loomo/odom
     * for a new Pose2D sample. Each sample is published once, stamped with its own time.
     */
    private fun publishBase() {
        if (!mSensor.isBind) {
            return
        }
        // No Loomo-clock stamps on the wire: wait for the first clock sample (TimeSync).
        if (!TimeSync.isReady()) {
            TimeSync.dropUnsynced()
            return
        }

        // The latest snapshot of each source; querySensorData takes no time argument.
        // Never add Sensor.BODY_IMU here: the Loomo does not implement it (see BaseTilt).
        val sensorData =
            mSensor.querySensorData(listOf(Sensor.POSE_2D, Sensor.BASE_IMU))
        val pose2DData = sensorData[0]
        val baseImuData = sensorData[1]

        val transforms = ArrayList<TransformStamped>(5)

        // Base IMU: platform tilt plus the range sensors' mounts, which are stamped with it.
        if (mBaseImuGate.isNew(baseImuData.timestamp)) {
            val stamp = jetsonStamp(baseImuData.timestamp, TimeSync.BASE_IMU)
            transforms.add(getUltrasonicTransform(stamp))
            transforms.add(getIrLeftTransform(stamp))
            transforms.add(getIrRightTransform(stamp))
            transforms.add(getBaseOdomTransform(baseImuData, stamp))
        }

        // Pose2D: loomo_odom -> base_odom (only with tf.odom=true) and /loomo/odom.
        val pose2D: Pose2D = mSensor.sensorDataToPose2D(pose2DData)
        val poseStamp: builtin_interfaces.msg.Time? =
            if (mPose2DGate.isNew(pose2D.timestamp)) jetsonStamp(pose2D.timestamp, TimeSync.POSE_2D) else null
        if (poseStamp != null && mPublishOdomTf) {
            transforms.add(getLoomoOdomTransform(pose2D, poseStamp))
        }

        if (transforms.isNotEmpty()) {
            val tfMsg: tf2_msgs.msg.TFMessage = tf2_msgs.msg.TFMessage()
            tfMsg.transforms = transforms
            mTfPublisher.publish(tfMsg)
        }

        if (poseStamp != null) {
            publishOdometry(pose2D, poseStamp)
        }
    }

    /** The head and camera transforms and the joint states at a depth frame's time [timestampUs]. */
    private fun publishFrameTf(timestampUs: Long) {
        // Bail out if we haven't bound to the sensor service yet
        if (!mSensor.isBind) {
            return
        }
        // No Loomo-clock stamps on the wire: wait for the first clock sample (TimeSync).
        if (!TimeSync.isReady()) {
            TimeSync.dropUnsynced()
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
                timestampUs,
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
                timestampUs,
                100 /* lookup threshold */
            )

        // neck_yaw_link -> realsense_color
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_RealsenseColor =
            AlgoTfRequest(
                Sensor.RS_COLOR_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                timestampUs,
                100 /* lookup threshold */
            )

        // neck_yaw_link -> realsense_depth
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_Fisheye =
            AlgoTfRequest(
                Sensor.RS_FE_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                timestampUs,
                100 /* lookup threshold */
            )

        // neck_yaw_link -> head pitch. Not published on /tf (robot_state_publisher draws the head
        // from /loomo/joint_states); it gives the joint state its pitch at the same instant as the
        // neck yaw above. HEAD_POSE_P_FRAME is HEAD_POSE_Y_FRAME moved up 0.104 m without rotation;
        // HEAD_POSE_P_R_FRAME adds the pitch, a pure rotation about Y (measured 2026-10-14).
        val T_NeckYawLink_HeadPitch =
            AlgoTfRequest(
                Sensor.HEAD_POSE_P_R_FRAME,
                Sensor.HEAD_POSE_Y_FRAME,
                timestampUs,
                100 /* lookup threshold */
            )

        val requests: List<AlgoTfRequest> = listOf(
            T_BaseLink_NeckLink,
            T_NeckYawLink_RealsenseDepth,
            T_NeckYawLink_RealsenseColor,
            T_NeckYawLink_Fisheye,
            T_NeckYawLink_HeadPitch
        )
        val results = mSensor.getMassiveTfData(requests)

        // The first four results go on /tf, every depth frame: they are looked up at the frame's
        // own time, so each capture is a new sample.
        var neckTf: TransformStamped? = null
        for ((i, r) in results.withIndex()) {
            if (i >= 4) break
            //val tfData = mSensor.getTfData(req.tgtFrameID, req.srcFrameID, req.timeStamp, 100)

            if (r.err_code == 0) {
                val t = algoTfDataToTransformStamped(r)
                if (i == 0) neckTf = t
                transforms.add(t)
            }
        }

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
        }

        if (transforms.size != 0) {
            // Fire off the completed transform message
            val tfMsg: tf2_msgs.msg.TFMessage = tf2_msgs.msg.TFMessage()
            tfMsg.transforms = transforms
            mTfPublisher.publish(tfMsg)
        }

        val headPitch = results.getOrNull(4)
        if (neckTf != null && headPitch != null && headPitch.err_code == 0) {
            publishJointState(neckTf.header.stamp, results[0], headPitch)
        } else {
            mJointStateSkips++
        }
    }

    companion object {
        /** The Pose2D and base IMU timer: 20 Hz, the rate at which Segway's services update them. */
        const val BASE_PERIOD_NS = 50_000_000L
    }

    private fun publishOdometry(pose2D: Pose2D, stamp: builtin_interfaces.msg.Time) {

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

        // Pose2D differences over a window (PoseDiffTwist), or the SDK's filtered velocities while
        // the window fills, after a stall, or with odom.twist=sdk.
        val diff = mPoseDiffTwist?.add(
            pose2D.timestamp, pose2D.x.toDouble(), pose2D.y.toDouble(), pose2D.theta.toDouble()
        )
        if (diff != null) {
            odomMsg.twist.twist.linear.x = diff.linear
            odomMsg.twist.twist.angular.z = diff.angular
        } else {
            if (mPoseDiffTwist != null) mTwistFallbacks++
            odomMsg.twist.twist.linear.x = pose2D.linearVelocity.toDouble()
            odomMsg.twist.twist.angular.z = pose2D.angularVelocity.toDouble()
        }

        // Intentionally not setting the mOdomMsg.twist.covariance matrix

        copyStamp(odomMsg.header.stamp, stamp)

        mOdometryPublisher.publish(odomMsg)
    }

    /**
     * Neck yaw and head pitch from one getMassiveTfData call at the depth frame's time, stamped
     * with that time ([stamp], already converted for the neck transform): 30 Hz, one stamp per
     * message. Previously the pitch and the stamp came from the 20 Hz HEAD_WORLD_IMU snapshot while
     * the yaw was looked up per depth frame, so ~32% of the messages repeated the previous stamp
     * with a different yaw, and tf2 listeners dropped robot_state_publisher's head transforms for
     * them (TF_REPEATED_DATA).
     *
     * HEAD_WORLD_IMU's pitch is the head's pitch in the world, not the joint angle the URDF wants
     * (the head hangs from base_link, which the base IMU already tilts). The TF pitch is the joint
     * angle: parked on 2026-10-14 it read 0.112 rad (nose down) while the SDK's HEAD_JOINT_PITCH
     * sensor read -0.111 (Segway's sign: positive is up) and HEAD_WORLD_IMU -0.002, so the robot
     * model's head now sits ~6 degrees lower than before.
     */
    private fun publishJointState(
        stamp: builtin_interfaces.msg.Time,
        T_BaseLink_NeckLink: AlgoTfData,
        T_NeckYawLink_HeadPitch: AlgoTfData
    ) {
        val jointStateMsg: sensor_msgs.msg.JointState = sensor_msgs.msg.JointState()

        var positions: DoubleArray = DoubleArray(2)

        positions[0] =
            T_BaseLink_NeckLink.q.rollRad.toDouble() // Head "Yaw". Segway calls rotation around the Z-axis "Roll"
        positions[1] =
            T_NeckYawLink_HeadPitch.q.yawRad.toDouble() // Head Pitch. Segway calls rotation around the Y-axis "Yaw"

        jointStateMsg.name = listOf("neck_yaw_joint", "head_pitch_joint")
        jointStateMsg.position = positions

        // My current converter code barfs if these arrays are not initialized to a real value
        jointStateMsg.velocity = DoubleArray(2) { 0.0 }
        jointStateMsg.effort = DoubleArray(2) { 0.0 }

        copyStamp(jointStateMsg.header.stamp, stamp)

        mJointStatePublisher.publish(jointStateMsg)
    }
}
