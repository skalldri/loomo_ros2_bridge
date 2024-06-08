package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.RobotAllSensors
import com.segway.robot.sdk.perception.sensor.Sensor
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch

class SensorInterface constructor(ctx: android.content.Context) {
    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private val TAG: String = "SensorIface"
    private var mIsPublishing: Boolean = false
    private var pollingJob: Job? = null
    private val pollingIntervalMs: Long = 10

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
                var allSensors: RobotAllSensors = mSensor.robotAllSensors

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

                // Publish head joint data

                delay(pollingIntervalMs)
            }
        }
        
        mIsPublishing = true
    }

    fun stopSensorPublisher() {
        // Raise exception if we already have a publisher for this camera type
        if (!mIsPublishing) {
            throw IllegalStateException("Not publishing sensors")
        }

        mIsPublishing = false
        pollingJob?.cancel()
    }
}