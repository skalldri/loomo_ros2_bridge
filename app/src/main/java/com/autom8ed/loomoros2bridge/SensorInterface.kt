package com.autom8ed.loomoros2bridge

import android.util.Log
import com.segway.robot.algo.tf.AlgoTfRequest
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.Sensor
import kotlinx.coroutines.delay

class SensorInterface constructor(ctx: android.content.Context) {

    private class SensorPublisher {

    }

    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()
    private val TAG: String = "SensorIface"
    private var mIsPublishing: Boolean = false
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
        
        mIsPublishing = true
    }

    fun stopSensorPublisher() {
        // Raise exception if we already have a publisher for this camera type
        if (!mIsPublishing) {
            throw IllegalStateException("Not publishing sensors")
        }

        mIsPublishing = false
    }
}