package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.RobotAllSensors
import com.segway.robot.sdk.perception.sensor.Sensor
import com.segway.robot.sdk.perception.sensor.SensorData
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.qos.QoSProfile
import tf2_msgs.msg.TFMessage
import java.lang.Math.toRadians
import java.util.Arrays
import java.util.concurrent.atomic.AtomicBoolean
import kotlin.concurrent.thread

class SensorInterface (ctx: android.content.Context, node: RosNode) {
    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private val TAG: String = "SensorIface"
    private var mIsPublishing: Boolean = false
    private var pollingJob: Job? = null
    private val pollingIntervalMs: Long = 10
    private var mThread: Thread? = null
    private val mThreadRun: AtomicBoolean = AtomicBoolean(true)
    private val mNode: RosNode = node

    private val mIrFov: Double = 5.0
    private val mUltrasonicFov: Double = 30.0

    private var mUltrasonicPublisher: Publisher<sensor_msgs.msg.Range>
    private var mIrLeftPublisher: Publisher<sensor_msgs.msg.Range>
    private var mIrRightPublisher: Publisher<sensor_msgs.msg.Range>

    init {
        mBindSensorListener = object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindHeadListener onBind() called")
            }

            override fun onUnbind(reason: String) {
                Log.d(TAG, "mBindHeadListener onUnbind() called with: reason = [$reason]")
            }
        }
        mSensor.bindService(ctx, mBindSensorListener)

        mUltrasonicPublisher = mNode.node.createPublisher(sensor_msgs.msg.Range::class.java, "/loomo/sensor/ultrasonic", QoSProfile.SENSOR_DATA)
        mIrLeftPublisher = mNode.node.createPublisher(sensor_msgs.msg.Range::class.java, "/loomo/sensor/ir_left", QoSProfile.SENSOR_DATA)
        mIrRightPublisher = mNode.node.createPublisher(sensor_msgs.msg.Range::class.java, "/loomo/sensor/ir_right", QoSProfile.SENSOR_DATA)

        start(10.0f)
    }

    fun start(rate_hz: Float) {
        stop()

        mThreadRun.set(true)
        mThread = thread() {
            while (mThreadRun.get()) {
                publishSensors()
                Thread.sleep((1000.0f / rate_hz).toLong())
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

    suspend fun startSensorPublisher(rate_hz: Float) {
        // Raise exception if we already have a publisher for this camera type
        if (mIsPublishing) {
            throw IllegalStateException("Already publishing sensors")
        }

        // Sensor interface must be started before we can start publishing
        // TODO: this is mega nasty. FIX THIS with some kind of threading once I'm
        // more familiar with Kotlin
        while (!mSensor.isBind) {
            delay(100);
        }

        pollingJob = CoroutineScope(Dispatchers.IO).launch {
            while (mIsPublishing) {

            }
        }

        mIsPublishing = true
    }

    fun publishSensors() {
        if (!mSensor.isBind) {
            return
        }

        val ultrasonicData = mSensor.ultrasonicDistance
        val infraredData = mSensor.infraredDistance

        // Publish Ultrasonic sensor_msgs/Range
        //
        // radiation_type = ULTRASOUND = 0
        // field_of_view = ??? // Need to measure this
        // min_range = ???
        // max_range = ???
        //
        // Range = +Inf for nothing detected
        //       = -Inf for object too close
        //       float, unit is meters
        // range = allSensors.ultrasonic.range

        // Publish Infrared x2 (Left, Right)
        //
        // radiation_type = INFRARED = 1
        // field_of_view = ??? // Need to measure this
        // min_range = ???
        // max_range = ???
        //
        // Range = +Inf for nothing detected
        //       = -Inf for object too close
        //       float, unit is meters
        // range = allSensors.ultrasonic.range

        val ussRangeMsg = sensor_msgs.msg.Range()
        ussRangeMsg.header.frameId = TfPublisherConstants.ULTRASONIC_FRAME_ID
        // TODO header.timestamp
        ussRangeMsg.radiationType = sensor_msgs.msg.Range.ULTRASOUND
        ussRangeMsg.fieldOfView = toRadians(mUltrasonicFov).toFloat() // 15 degree cone, TODO: total guess
        ussRangeMsg.minRange = 0.3f // Approximately measure, min value seen from Loomo API ~24cm
        ussRangeMsg.maxRange = 1.5f // Max value seen from Loomo API
        ussRangeMsg.range = (ultrasonicData.distance / 1000.0f) // Loomo returns in mm, ROS expects m
        mUltrasonicPublisher.publish(ussRangeMsg)

        val irLeftRangeMsg = sensor_msgs.msg.Range()
        irLeftRangeMsg.header.frameId = TfPublisherConstants.IR_LEFT_FRAME_ID
        // TODO header.timestamp
        irLeftRangeMsg.radiationType = sensor_msgs.msg.Range.INFRARED
        irLeftRangeMsg.fieldOfView = toRadians(mIrFov).toFloat() // 30 degree cone, TODO: total guess
        irLeftRangeMsg.minRange = 0.2f // 20cm, TODO: total guess
        irLeftRangeMsg.maxRange = 5f // 5m, TODO: total guess
        irLeftRangeMsg.range = (infraredData.leftDistance / 1000.0f) // Loomo returns in mm, ROS expects m
        mIrLeftPublisher.publish(irLeftRangeMsg)

        val irRightRangeMsg = sensor_msgs.msg.Range()
        irRightRangeMsg.header.frameId = TfPublisherConstants.IR_RIGHT_FRAME_ID
        // TODO header.timestamp
        irRightRangeMsg.radiationType = sensor_msgs.msg.Range.INFRARED
        irRightRangeMsg.fieldOfView =
            toRadians(mIrFov).toFloat() // 30 degree cone, TODO: total guess
        irRightRangeMsg.minRange = 0.2f // 20cm, TODO: total guess
        irRightRangeMsg.maxRange = 5f // 5m, TODO: total guess
        irRightRangeMsg.range = (infraredData.rightDistance / 1000.0f) // Loomo returns in mm, ROS expects m
        mIrRightPublisher.publish(irRightRangeMsg)
    }

    fun getBaseImu(): SensorData {
        val sensorData: SensorData = mSensor.querySensorData(listOf(Sensor.BASE_IMU))[0]
        return sensorData
    }

    fun getHeadImu(): SensorData {
        val sensorData: SensorData = mSensor.querySensorData(listOf(Sensor.HEAD_WORLD_IMU))[0]
        return sensorData
    }
}