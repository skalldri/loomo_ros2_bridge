package com.autom8ed.lr2;

import android.content.Context;
import android.os.RemoteException;
import android.support.annotation.IntDef;

import com.segway.robot.sdk.base.bind.ForegroundBindController;
import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.locomotion.head.Angle;
import com.segway.robot.sdk.locomotion.head.AngularVelocity;
import com.autom8ed.lr2.RobotHeadManager;
import com.segway.robot.sdk.locomotion.head.RobotHeadException;

import java.util.concurrent.atomic.AtomicBoolean;

/**
 * The manager is used to manage the robot's head. This class contains all of
 * the head controls and data APIs.
 */
public class Head {

    /**
     * Smooth tracking mode.
     * This is the default mode. When in this mode, the robot keeps its head in a stable pose
     * that relative to the robot base. And {@link com.segway.robot.sdk.locomotion.head.Head#setPitchAngularVelocity(float)} and
     * {@link com.segway.robot.sdk.locomotion.head.Head#setYawAngularVelocity(float)} won't make the robot head to move with the setting
     * velocity. Use other setting methods to control head pose.
     */
    public static final int MODE_SMOOTH_TACKING = 1 << 2;

    /**
     * The orientation lock mode.
     * When in this mode, the robot keeps its head yaw in a stable pose which is relative to the
     * world coordinate. Use other velocity setting methods to control head pose.
     */
    public static final int MODE_ORIENTATION_LOCK = 1 << 3;

    /**
     * Head free mode.
     */
    public static final int MODE_EMOJI = 3 << 16;

    private static com.autom8ed.lr2.Head ourInstance = new com.autom8ed.lr2.Head();
    private static ForegroundBindController mController = new ForegroundBindController();
    private com.autom8ed.lr2.RobotHeadManager mRobotHeadManager;
    private final AtomicBoolean isControllerSet = new AtomicBoolean(false);

    private Head() {
        mRobotHeadManager = new com.autom8ed.lr2.RobotHeadManager();
    }

    /**
     * get the instance of {@link com.segway.robot.sdk.locomotion.head.Head}.
     *
     * @return a Head instance.
     */
    public static com.autom8ed.lr2.Head getInstance() {
        return ourInstance;
    }

    /**
     * Connect to the robot head service. This method must be called before using the head control APIs.
     *
     * @param context  any Android context.
     * @param listener the listener of bind state.
     * @return true if bind service successfully, otherwise false.
     */
    public final boolean bindService(Context context, ServiceBinder.BindStateListener listener) {
        context = context.getApplicationContext();
        boolean ret = mRobotHeadManager.bindService(context, listener);
        if (ret) {
            synchronized (isControllerSet) {
                if (!isControllerSet.get()) {
                    isControllerSet.set(true);
                    mController.init(context, mRobotHeadManager);
                }
            }
        }
        return ret;
    }

    /**
     * Disconnect from the robot head service.
     */
    public void unbindService() {
        mRobotHeadManager.unbindService();
    }

    /**
     * Get the robot motion mode.
     *
     * @return mode the robot motion mode.
     */
    public int getMode() {
        return mRobotHeadManager.getMode();
    }

    /**
     * Set the robot motion mode.
     *
     * @param mode the motion mode to be set.
     */
    public void setMode(@HeadMode int mode) {
        mRobotHeadManager.setMode(mode);
    }

    /**
     * Reset the robot head yaw direction angle respective to base frame and
     * reset the robot head pitch direction to the world horizontal pose.
     */
    public void resetOrientation() {
        mRobotHeadManager.resetOrientation();
    }

    /**
     * Get the robot head yaw angle {@link Angle} relative to the world coordinate.
     *
     * @return the {@link Angle}, the angle value is ranged from -{@link Math#PI} to
     * {@link Math#PI}. Now it always returns 0.
     */
    public Angle getWorldYaw() {
        return mRobotHeadManager.getWorldYaw();
    }

    /**
     * Set the robot head yaw angle which is relative to the world coordinate.
     * Unusable in {@link com.segway.robot.sdk.locomotion.head.Head#MODE_ORIENTATION_LOCK} mode.
     *
     * @param yaw The value is ranged from -{@link Math#PI} to {@link Math#PI}.
     */
    public void setWorldYaw(float yaw) {
        mRobotHeadManager.setWorldYaw(yaw);
    }

    /**
     * Get the robot head pitch {@link Angle} which is relative to the world coordinate.
     *
     * @return the {@link Angle} the value is ranged -{@link Math#PI} to {@link Math#PI}.
     */
    public Angle getWorldPitch() {
        return mRobotHeadManager.getWorldPitch();
    }

    /**
     * Set the robot head pitch angle which is relative to the world coordinate.
     *
     * @param pitch the pitch {@link Angle} to be set, the value is ranged from -{@link Math#PI} to
     *              {@link Math#PI}.
     */
    public void setWorldPitch(float pitch) {
        mRobotHeadManager.setWorldPitch(pitch);
    }

    /**
     * Get the robot head roll {@link Angle} which is relative to the world coordinate.
     *
     * @return the roll {@link Angle} the angle value is ranged from from -{@link Math#PI} to {@link Math#PI}.
     */
    public Angle getWorldRoll() {
        return mRobotHeadManager.getWorldRoll();
    }

    /**
     * Set the incremental yaw which is relative to the robot current head yaw.
     *
     * @param yaw the incremental yaw to be set, its value is ranged from -{@link Math#PI} to {@link Math#PI}.
     */
    public void setIncrementalYaw(float yaw) {
        mRobotHeadManager.setIncrementalYaw(yaw);
    }

    /**
     * Set the incremental pitch which is relative to the robot current head pitch.
     *
     * @param pitch The incremental yaw to be set, its value is ranged from -{@link Math#PI} to
     *              {@link Math#PI}.
     */
    public void setIncrementalPitch(float pitch) {
        mRobotHeadManager.setIncrementalPitch(pitch);
    }

    /**
     * Get the robot head yaw moving velocity.
     *
     * @return the {@link AngularVelocity}.
     */
    public AngularVelocity getYawAngularVelocity() {
        return mRobotHeadManager.getYawAngularVelocity();
    }

    /**
     * Set the head yaw angular velocity.
     *
     * @param yawAngularVelocity the velocity to be set, its unit is rad per second.
     */
    public void setYawAngularVelocity(float yawAngularVelocity) {
        mRobotHeadManager.setYawAngularVelocity(yawAngularVelocity);
    }

    /**
     * Get the robot head pitch moving velocity.
     *
     * @return the {@link AngularVelocity}.
     */
    public AngularVelocity getPitchAngularVelocity() {
        return mRobotHeadManager.getPitchAngularVelocity();
    }

    /**
     * Set the head pitch angular velocity.
     *
     * @param pitchAngularVelocity the velocity to be set, its unit is rad per second.
     */
    public void setPitchAngularVelocity(float pitchAngularVelocity) {
        mRobotHeadManager.setPitchAngularVelocity(pitchAngularVelocity);
    }

    /**
     * Get the robot head roll moving velocity.
     *
     * @return the {@link AngularVelocity}.
     */
    public AngularVelocity getRollAngularVelocity() {
        return mRobotHeadManager.getRollAngularVelocity();
    }

    /**
     * Set the robot head state.
     *
     * @param listener
     */
    public void setHeadStateListener(com.autom8ed.lr2.Head.HeadState listener) {
        // TODO: 16/9/5
    }

    @IntDef(value = {MODE_SMOOTH_TACKING, MODE_ORIENTATION_LOCK, MODE_EMOJI})
    @interface
    HeadMode {
    }

    /**
     * the head state value.
     */
    interface HeadState {
        int STATE_NORMAL = 0;
        int STATE_LOCKING = 1;
        int STATE_LIFT_UP = 2;
        int STATE_STUCK = 3;
    }

    /**
     * Get the robot yaw {@link Angle} which is relative to the robot base coordinate.
     * Use {@link com.segway.robot.sdk.locomotion.head.Head#getHeadJointYaw()} replace.
     *
     * @return the {@link Angle}.
     */
    @Deprecated
    public Angle getYawRespectBase() {
        return mRobotHeadManager.getYawRespectBase();
    }

    /**
     * Get the robot pitch {@link Angle} which is relative to the robot base coordinate.
     * Use {@link com.segway.robot.sdk.locomotion.head.Head#getHeadJointPitch()} replace.
     *
     * @return the {@link Angle}.
     */
    @Deprecated
    public Angle getPitchRespectBase() {
        return mRobotHeadManager.getPitchRespectBase();
    }

    /**
     * Get the robot roll {@link Angle} which is relative to the robot base coordinate.
     * Use {@link com.segway.robot.sdk.locomotion.head.Head#getHeadJointRoll()} replace.
     *
     * @return the {@link Angle}.
     */
    @Deprecated
    public Angle getRollRespectBase() {
        return mRobotHeadManager.getRollRespectBase();
    }

    /**
     * Set the head yaw which is relative to the robot base coordinate.
     * Use {@link com.segway.robot.sdk.locomotion.head.Head#setHeadJointYaw(float)} replace
     * @param yaw the yaw angle to be set.
     */
    @Deprecated
    public void setYawRespectBase(float yaw) {
        mRobotHeadManager.setYawRespectBase(yaw);
    }


    /**
     * Set the head yaw which is relative to the robot base coordinate.
     *
     * @param yaw the yaw angle to be set.
     */
    public void setHeadJointYaw(float yaw) {
        mRobotHeadManager.setYawRespectBase(yaw);
    }
    /**
     * Get the robot pitch {@link Angle} which is relative to the robot base coordinate.
     *
     * @return the {@link Angle}.
     */
    public Angle getHeadJointPitch() {
        return mRobotHeadManager.getPitchRespectBase();
    }

    /**
     * Get the robot roll {@link Angle} which is relative to the robot base coordinate.
     *
     * @return the {@link Angle}.
     */
    public Angle getHeadJointRoll() {
        return mRobotHeadManager.getRollRespectBase();
    }

    /**
     * Get the robot roll {@link Angle} which is relative to the robot base coordinate.
     *
     * @return the {@link Angle}.
     */
    public Angle getHeadJointYaw() {
        return mRobotHeadManager.getYawRespectBase();
    }

    /**
     * Set the head light mode.
     *
     * @param mode the mode to be set. The value is ranged from 0 to 13.
     */
    public void setHeadLightMode(int mode) {
        mRobotHeadManager.setHeadLightMode(mode);
    }


    /**
     * Get the state of binding.
     *
     * @return the state of binding.
     */
    public boolean isBind() {
        return mRobotHeadManager.isBind();
    }

    /**
     * Set the yaw + pitch of the head simultaneously
     *
     * @param yaw neck yaw angle in radians
     * @param pitch head pitch angle in radians
     */
    public void setJointPosition(float yaw, float pitch) { mRobotHeadManager.setJointPosition(yaw, pitch); }

    public void setJointYaw(float yaw) { mRobotHeadManager.setJointYaw(yaw); }

    public void setJointPitch(float pitch) { mRobotHeadManager.setJointPitch(pitch); }
}
