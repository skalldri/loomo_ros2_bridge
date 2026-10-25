package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder.BindStateListener
import com.segway.robot.sdk.locomotion.sbv.Base
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.qos.QoSProfile
import org.ros2.rcljava.qos.policies.Durability
import org.ros2.rcljava.qos.policies.Reliability


/**
 * The wheels, as [DriveGate] commands them: the Segway base in raw mode.
 *
 * In raw mode the SDK's setLinearVelocity and setAngularVelocity each relay one value over binder
 * to Segway's locomotion service and return; the SDK keeps no timer and resends nothing, and
 * Base.stop() is just both set to zero. So nothing in the SDK stops the base when commands stop;
 * DriveGate's timeouts send the zero.
 */
class BaseWheels : DriveGate.Wheels {
    private val mBase: Base = Base.getInstance()

    override fun set(linear: Double, angular: Double): Boolean {
        if (!mBase.isBind) return false
        return try {
            mBase.setLinearVelocity(linear.toFloat())
            mBase.setAngularVelocity(angular.toFloat())
            true
        } catch (e: RuntimeException) {
            Log.e(TAG, "the base did not take linear=$linear angular=$angular", e)
            false
        }
    }

    companion object {
        const val TAG = "LocomotionPlatformIface"
    }
}

class LocomotionPlatformInterface(ctx: android.content.Context, node: RosNode, gate: DriveGate) {
    // get Locomotion SDK instance
    private var mBase: Base = Base.getInstance();
    private var mBindLocomotionListener: BindStateListener;
    val TAG: String = "LocomotionPlatformIface"
    val mNode: RosNode = node
    private val mGate: DriveGate = gate

    // The push-to-start state, for a stop gate on the ROS side and for RViz. Transient local, so a
    // late reader gets the current state at once; DriveStateRepeater also resends it every second.
    // Publish-only: nothing the app subscribes to can switch it to driving.
    private val mDriveStatePublisher: Publisher<std_msgs.msg.Bool> =
        mNode.node.createPublisher(
            std_msgs.msg.Bool::class.java,
            DRIVE_STATE_TOPIC,
            QoSProfile.keepLast(1)
                .setReliability(Reliability.RELIABLE)
                .setDurability(Durability.TRANSIENT_LOCAL)
        )
    private val mDriveStateRepeater = DriveStateRepeater { driving ->
        val msg = std_msgs.msg.Bool()
        msg.data = driving
        mDriveStatePublisher.publish(msg)
    }

    init {
        mBindLocomotionListener = object : BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindLocomotionListener onBind() called")

                // Configure Base to accept raw linear/angular velocity commands. Raw mode keeps
                // the base balancing; DriveGate only ever changes the velocities.
                mBase.setControlMode(Base.CONTROL_MODE_RAW);

                // A zero unless driving, in case an earlier run of the app left the base moving.
                mGate.baseBound()
            }

            override fun onUnbind(reason: String) {
                Log.e(TAG, "onUnbind() called with: reason = [$reason]")
            }
        }

        // Connect to the service
        mBase.bindService(ctx, mBindLocomotionListener);

        // /cmd_vel reaches the wheels only through the gate: only while driving, and the gate
        // zeroes the wheels when the messages stop.
        mNode.node.createSubscription(
            geometry_msgs.msg.Twist::class.java,
            "/cmd_vel"
        ) { msg: geometry_msgs.msg.Twist ->
            mGate.cmdVel(msg.linear.x, msg.angular.z)
        }

        mNode.node.createSubscription(
            std_msgs.msg.Bool::class.java,
            "/loomo/body_light"
        ) { msg: std_msgs.msg.Bool ->
            if (mBase.isBind) {
                mBase.enableBodyLight(msg.data)
            }
        }
    }

    /** Publishes the driving state on a change and every second; called on every gate tick. */
    fun publishDriveState(driving: Boolean, nowNs: Long) {
        try {
            mDriveStateRepeater.update(driving, nowNs)
        } catch (t: Throwable) {
            Log.e(TAG, "publishing $DRIVE_STATE_TOPIC failed", t)
        }
    }

    companion object {
        /** std_msgs/Bool: true while the app passes /cmd_vel to the wheels (push to start). */
        const val DRIVE_STATE_TOPIC = "/loomo/drive_enabled"
    }
}
