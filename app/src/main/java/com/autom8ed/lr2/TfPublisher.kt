package com.autom8ed.loomoros2bridge

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.perception.sensor.Sensor

import org.ros2.rcljava.RCLJava;

import org.ros2.android.activity.ROSActivity;
import org.ros2.rcljava.node.BaseComposableNode

class TfPublisher(ctx: android.content.Context) {
    private var mBindSensorListener: ServiceBinder.BindStateListener;
    private var mSensor: Sensor = Sensor.getInstance()

    private val TAG: String = "TfPublisher"

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

    fun indicateTfNeededAtTime()
    {

    }
}