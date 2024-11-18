package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.autom8ed.lr2.Head;

class HeadInterface(ctx: android.content.Context, node: RosNode) {
    private var mBindHeadListener: ServiceBinder.BindStateListener;
    private var mHead: Head = Head.getInstance()
    private val mNode: RosNode = node
    private val TAG: String = "HeadIface"
    private var mHeadPitch: Float = 0f

    // Loomo SDK doesn't define these values...
    private val HEAD_LIGHT_MODE_MIN = 0
    private val HEAD_LIGHT_MODE_MAX = 13

    // Documenting head light mode
    val HEAD_LIGHT_MODE_OFF = 0
    val HEAD_LIGHT_MODE_BLUE_STATIC = 1
    val HEAD_LIGHT_MODE_BLUE_SPINNING = 2
    val HEAD_LIGHT_MODE_BLUE_WHITE_SPINNING = 3
    val HEAD_LIGHT_MODE_BLUE_WHITE_ALTERNATING = 4
    val HEAD_LIGHT_MODE_RED_FLASHING = 5
    val HEAD_LIGHT_MODE_GREEN_FLASHING = 6
    val HEAD_LIGHT_MODE_GREEN_FADE_IN_OUT = 7
    val HEAD_LIGHT_MODE_ORANGE_FLASHING = 8
    val HEAD_LIGHT_MODE_BLUE_FADE_IN_OUT = 9
    val HEAD_LIGHT_MODE_PURPLE_WHITE_SPINNING = 10
    val HEAD_LIGHT_MODE_PURPLE_WHITE_ALTERNATING = 11
    val HEAD_LIGHT_MODE_BLUE_PULSING = 12
    val HEAD_LIGHT_MODE_WHITE_LOADING = 13

    init {
        mBindHeadListener = object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindHeadListener onBind() called")

                mHead.mode = Head.MODE_SMOOTH_TACKING
                resetHeadOrientation()
                mHead.setHeadLightMode(HEAD_LIGHT_MODE_WHITE_LOADING)
            }

            override fun onUnbind(reason: String) {
                Log.d(TAG, "mBindHeadListener onUnbind() called with: reason = [$reason]")
            }
        }
        mHead.bindService(ctx, mBindHeadListener)

        mNode.node.createSubscription(std_msgs.msg.UInt8::class.java, "/loomo/head/light_mode") { msg: std_msgs.msg.UInt8 ->
            if (mHead.isBind) {
                setLightMode(msg.data.toInt())
            }
        }

        mNode.node.createSubscription(geometry_msgs.msg.QuaternionStamped::class.java, "/loomo/head/orientation") { msg: geometry_msgs.msg.QuaternionStamped ->
            if (!mHead.isBind) {
                return@createSubscription;
            }

            val quaternion = com.segway.robot.algo.tf.Quaternion(msg.quaternion.x.toFloat(), msg.quaternion.y.toFloat(), msg.quaternion.z.toFloat(), msg.quaternion.w.toFloat())

            val yaw: Float = quaternion.yawRad
            val pitch: Float = quaternion.rollRad // Segway's axis convention doesn't match to the hardware names

            mHead.setHeadJointYaw(yaw)

            val mHeadPitchDelta: Float = pitch - mHead.headJointPitch.angle

            // Segway forgot to expose an API for setting the head-pitch directly...
            // Nice...
            mHead.setIncrementalPitch(mHeadPitchDelta)
        }
    }

    fun resetHeadOrientation() {
        mHead.resetOrientation()
    }

    fun setLightMode(mode: Int) {
        if (mode in HEAD_LIGHT_MODE_MIN..HEAD_LIGHT_MODE_MAX) {
            mHead.setHeadLightMode(mode)
        }
    }
}