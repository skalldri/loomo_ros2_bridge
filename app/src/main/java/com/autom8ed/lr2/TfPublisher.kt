package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.algo.tf.AlgoTfData
import com.segway.robot.algo.tf.AlgoTfRequest
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.Sensor
import geometry_msgs.msg.Quaternion
import geometry_msgs.msg.Transform
import geometry_msgs.msg.TransformStamped
import geometry_msgs.msg.Vector3
import org.ros2.rcljava.publisher.Publisher
import java.util.Arrays
import java.util.concurrent.BlockingQueue
import java.util.concurrent.LinkedBlockingDeque
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread
import kotlin.math.cos
import kotlin.math.sin


object TfPublisherConstants {
    val BASE_ODOM = "base_odom"
    val BASE_LINK = "base_link"
    val NECK_LINK = "neck_link"
    val FISHEYE_FRAME_ID = "loomo_fisheye"
    val FISHEYE_OPTICAL_FRAME_ID = "loomo_fisheye_optical"
    val REALSENSE_DEPTH_FRAME_ID = "looomo_realsense_depth"
    val REALSENSE_COLOR_FRAME_ID = "looomo_realsense_color"
    val REALSENSE_DEPTH_OPTICAL_FRAME_ID = "looomo_realsense_depth_optical"
    val REALSENSE_COLOR_OPTICAL_FRAME_ID = "looomo_realsense_color_optical"
    val ULTRASONIC_FRAME_ID = "loomo_ultrasonic"
    val IR_LEFT_FRAME_ID = "loomo_ir_left"
    val IR_RIGHT_FRAME_ID = "loomo_ir_right"

    val ULTRASONIC_HEIGHT_M = 0.44
    val ULTRASONIC_FORWARD_M = 0.1

    val IR_CENTER_OFFSET_M = 0.025
    val IR_HEIGHT_OFFSET_FROM_USS_M = -0.035
}

class TfPublisher(ctx: android.content.Context, node: RosNode) {
    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private var mTfPublisher: Publisher<tf2_msgs.msg.TFMessage>
    private val mNode: RosNode = node
    private var mThread: Thread? = null
    private val mThreadRun: AtomicBoolean = AtomicBoolean(true)
    private val mTimestampQueue: BlockingQueue<Long> = LinkedBlockingDeque<Long>()

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

        start()
    }

    fun indicateTfNeededAtTime(timestamp: Long)
    {
        mTimestampQueue.add(timestamp)
    }

    fun start() {
        stop()

        mThreadRun.set(true)
        mThread = thread() {
            while (mThreadRun.get()) {
                val timestamp: Long = mTimestampQueue.take()
                publishTf(timestamp)
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
        val t: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    private fun getLoomoToOpticalFrameTf(tgtFrame: String, srcFrame: String): TransformStamped {
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

        val t = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    private fun getUltrasonicTransform(): TransformStamped {
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

        val t = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    private fun getIrLeftTransform(): TransformStamped {
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

        val t = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    private fun getIrRightTransform(): TransformStamped {
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

        val t = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    private fun toQuaternion(roll: Double, pitch: Double, yaw: Double): Quaternion // roll (x), pitch (y), yaw (z), angles are in radians
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
    private fun getBaseOdomTransform(): TransformStamped {
        val mBaseImuData =
            mSensor.querySensorData(listOf(Sensor.BASE_IMU))[0]
        Log.i(TAG, "Base IMU Sensor: ${mBaseImuData.floatData[0]}, ${mBaseImuData.floatData[1]}, ${mBaseImuData.floatData[2]}")

        val mWorldImuData =
            mSensor.querySensorData(listOf(Sensor.HEAD_WORLD_IMU))[0]
        Log.i(TAG, "World IMU Sensor: ${mWorldImuData.floatData[0]}, ${mWorldImuData.floatData[1]}, ${mWorldImuData.floatData[2]}")

        val vector3 = Vector3()

        vector3.setX(0.0)
        vector3.setY(0.0)
        vector3.setZ(0.0)

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
        transformStamped.setChildFrameId(TfPublisherConstants.BASE_LINK)
        transformStamped.header.setFrameId(TfPublisherConstants.BASE_ODOM)

        val t = builtin_interfaces.msg.Time()

        // TODO: timestamping
        t.sec = 0
        t.nanosec = 0

        transformStamped.header.setStamp(t)
        return transformStamped
    }

    fun publishTf(timestamp: Long) {
        // Bail out if we haven't bound to the sensor service yet
        if (!mSensor.isBind) {
            return
        }

        // TODO: calculate initial arraylist capacity based on expected robot transforms
        val transforms: MutableList<geometry_msgs.msg.TransformStamped> = MutableList(0) { geometry_msgs.msg.TransformStamped() }

        // base_odom -> base_link
        // base_odom is base_link minus the rotation of the platform
        // val T_BaseOdom_BaseLink =
        //     AlgoTfRequest(Sensor.BASE_POSE_FRAME, Sensor.BASE_ODOM_FRAME, timestamp, 100 /* lookup threshold */)

        // base_link -> neck_link
        // neck_link is in the center of the neck, without any rotation component
        val T_BaseLink_NeckLink =
            AlgoTfRequest(Sensor.HEAD_POSE_Y_FRAME, Sensor.BASE_POSE_FRAME, timestamp, 100 /* lookup threshold */)

        // neck_link -> neck_yaw_link
        // neck_yaw_link has the same location as neck_link, but it incorporates the rotation of the neck
        //val T_NeckLink_NeckYawLink =
        //    AlgoTfRequest(Sensor.HEAD_POSE_Y_FRAME, Sensor.NECK_POSE_FRAME, timestamp, 100 /* lookup threshold */)

        // neck_yaw_link -> realsense_depth
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_RealsenseDepth =
            AlgoTfRequest(Sensor.RS_DEPTH_FRAME, Sensor.HEAD_POSE_Y_FRAME, timestamp, 100 /* lookup threshold */)

        // neck_yaw_link -> realsense_color
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_RealsenseColor =
            AlgoTfRequest(Sensor.RS_COLOR_FRAME, Sensor.HEAD_POSE_Y_FRAME, timestamp, 100 /* lookup threshold */)

        // neck_yaw_link -> realsense_depth
        // The realsense camera is mounted on the yaw-only portion of the head
        val T_NeckYawLink_Fisheye =
            AlgoTfRequest(Sensor.RS_FE_FRAME, Sensor.HEAD_POSE_Y_FRAME, timestamp, 100 /* lookup threshold */)

        val requests: List<AlgoTfRequest> = listOf(/*T_BaseOdom_BaseLink,*/ T_BaseLink_NeckLink, T_NeckYawLink_RealsenseDepth, T_NeckYawLink_RealsenseColor, T_NeckYawLink_Fisheye)
        val results = mSensor.getMassiveTfData(requests)

        for (r in results) {
            //val tfData = mSensor.getTfData(req.tgtFrameID, req.srcFrameID, req.timeStamp, 100)

            if (r.err_code == 0) {
                transforms.add(algoTfDataToTransformStamped(r))
            }
        }

        if (transforms.size != 0) {
            // Add static / fixup transforms
            transforms.add(getLoomoToOpticalFrameTf(TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID, TfPublisherConstants.REALSENSE_DEPTH_FRAME_ID))
            transforms.add(getLoomoToOpticalFrameTf(TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID, TfPublisherConstants.REALSENSE_COLOR_FRAME_ID))
            transforms.add(getLoomoToOpticalFrameTf(TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID, TfPublisherConstants.FISHEYE_FRAME_ID))
            transforms.add(getUltrasonicTransform())
            transforms.add(getIrLeftTransform())
            transforms.add(getIrRightTransform())

            // getBaseOdomTransform()

            // Fire off the completed transform message
            val tfMsg: tf2_msgs.msg.TFMessage = tf2_msgs.msg.TFMessage()
            tfMsg.transforms = transforms
            mTfPublisher.publish(tfMsg)
        }
    }
}