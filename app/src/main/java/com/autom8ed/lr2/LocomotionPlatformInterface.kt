package com.autom8ed.loomoros2bridge

import android.R.id.message
import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder.BindStateListener
import com.segway.robot.sdk.locomotion.sbv.Base


class LocomotionPlatformInterface constructor(ctx: android.content.Context, node: RosNode) {
    // get Locomotion SDK instance
    private var mBase: Base = Base.getInstance();
    private var mBindLocomotionListener: BindStateListener;
    val TAG: String = "LocomotionPlatformIface"
    val mNode: RosNode = node

    init {
        mBindLocomotionListener = object : BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindLocomotionListener onBind() called")

                // Configure Base to accept raw linear/angular velocity commands
                mBase.setControlMode(Base.CONTROL_MODE_RAW);

                // mLocomotionSubscriber.loomo_started(mBase)
                // mTFPublisher.loomo_started(mBase)

                // Try a call to start listening, this may fail is ROS is not started yet (which is fine)
                // TODO: check state of checkbox
                //if (mSubMotionSwitch.isChecked()) {
                //    mLocomotionSubscriber.start()
                //}

                //if (mPubTFSwitch.isChecked()) {
                //    mTFPublisher.start()
                //}
            }

            override fun onUnbind(reason: String) {
                Log.d(TAG, "onUnbind() called with: reason = [$reason]")
            }
        }

        // Connect to the service
        mBase.bindService(ctx, mBindLocomotionListener);

        // Subscribe to the /cmd_vel ros topic
        this.mNode.node.createSubscription(geometry_msgs.msg.Twist::class.java, "cmd_vel") { msg: geometry_msgs.msg.Twist ->
            // Only submit messages if we're connected to the service
            if (mBase.isBind) {
                mBase.setLinearVelocity(msg.linear.x.toFloat())
                mBase.setAngularVelocity(msg.angular.z.toFloat())
            }
        }
    }
}