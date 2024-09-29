package com.autom8ed.lr2;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.IBinder;
import android.os.RemoteException;

import com.segway.robot.sdk.base.bind.ServiceBinder;
import com.segway.robot.sdk.base.log.Logger;
import com.segway.robot.sdk.locomotion.head.Angle;
import com.segway.robot.sdk.locomotion.head.AngularVelocity;
import com.segway.robot.sdk.locomotion.head.IRobotHeadService;
import com.segway.robot.sdk.locomotion.head.RobotHeadException;

/**
 *
 */
class RobotHeadManager implements ServiceBinder {
    private static final String TAG = "RobotHeadManager";
    private static final String SERVICE_CLASS_NAME = "com.segway.robot.host.coreservice.locomotionservice.RobotHeadService";
    private static final String SERVICE_PACKAGE_NAME = "com.segway.robot.host.coreservice.locomotionservice";
    IRobotHeadService mRobotHeadService;
    private Context mContext;
    private boolean isBind = false;
    private BindStateListener mBindStateListener;
    private ServiceConnection mServiceConnection = new ServiceConnection() {
        /*
         * check any unexpected potential errors
         * */
        @Override
        public void onServiceConnected(ComponentName name, IBinder service) {
            mRobotHeadService = IRobotHeadService.Stub.asInterface(service);
            if (mRobotHeadService == null) {
                bindError(new RuntimeException("Product not supported"));
            }
            // check version
//            Version version = VersionInfo.getVersion();
//            try {
//                Version serviceVersion = mRobotHeadService.getVersion();
//                version.check("VersionInfo", serviceVersion);
//            } catch (RemoteException e) {
//                String error = "Cannot get Service version, err = " + e.getMessage();
//
//                // disconnect to remote service
//                bindError(e, error);
//                return;
//            } catch (VersionMismatchException e) {
//                String error = "Version mismatch: " + e.getMessage();
//                // disconnect to remote service
//                bindError(e, error);
//                return;
//            }
            try {
                mRobotHeadService.registerWatcher(service, mContext.getPackageName());
            } catch (RemoteException e) {
                bindError(e);
                return;
            }
            mBindStateListener.onBind();
            isBind = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName name) {
            mRobotHeadService = null;
            mBindStateListener.onUnbind("Service disconnected");
            isBind = false;
        }
    };

    protected RobotHeadManager() {
    }

    @Override
    public synchronized boolean bindService(Context context, BindStateListener listener) {
        if (context == null || listener == null) {
            throw new IllegalArgumentException("context and listener can't null");
        }
        mBindStateListener = listener;
        if (isBind) {
            return true;
        }
        mContext = context.getApplicationContext();
        return connectService(mContext);
    }

    private boolean connectService(Context context) {
        Intent serviceIntent = new Intent();
        serviceIntent.setAction(SERVICE_CLASS_NAME);
        serviceIntent.setPackage(SERVICE_PACKAGE_NAME);
        serviceIntent.putExtra("level", "internal");
        serviceIntent.setClassName(SERVICE_PACKAGE_NAME, SERVICE_CLASS_NAME);
        return context.bindService(serviceIntent, mServiceConnection, Context.BIND_AUTO_CREATE);
    }

    @Override
    public synchronized void unbindService() {
        if (!isBind) {
            return;
        }
        try {
            mRobotHeadService.unregisterWatcher(mContext.getPackageName());
        } catch (RemoteException e) {
            e.printStackTrace();
            Logger.e(TAG, "unregisterWatcher fail", e);
        }
        mContext.unbindService(mServiceConnection);
        isBind = false;
    }

    @Override
    public boolean isBind() {
        return isBind;
    }

    @Override
    public BindStateListener getBindStateListener() {
        return mBindStateListener;
    }

    /**
     * Get the angle that is relative to the world
     */
    public Angle getWorldYaw() {
        try {
            return mRobotHeadService.getWorldYaw();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getWorldYaw error", e);
        }
    }

    /*
     * Unusable in MODE_SMOOTH_TACKING
     */
    public void setWorldYaw(float yaw) {
        try {
            mRobotHeadService.setWorldYaw(yaw);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setWorldYaw error", e);
        }
    }

    public Angle getWorldPitch() {
        try {
            return mRobotHeadService.getWorldPitch();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getWorldPitch error", e);
        }
    }

    public void setWorldPitch(float pitch) {
        // TODO: 16/9/5
        try {
            mRobotHeadService.setWorldPitch(pitch);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setWorldPitch error", e);
        }
    }

    public Angle getWorldRoll() {
        try {
            return mRobotHeadService.getWorldRoll();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getWorldRoll error", e);
        }
    }

    /*
     * Unusalbe in MODE_ORIENTATION_LOCK
     */
    public void setIncrementalYaw(float yaw) {
        try {
            mRobotHeadService.setIncrementalYaw(yaw);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setIncrementalYaw error", e);
        }
    }

    /**
     * Reset Yaw respect to base
     * Reset Pitch respect to world
     */
    public void resetOrientation() {
        try {
            mRobotHeadService.resetOrientation();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("resetOrientation error", e);
        }
    }

    /**
     * Get mode
     *
     * @return mode
     */
    public int getMode() {
        try {
            return mRobotHeadService.getMode();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getMode error", e);
        }
    }

    /**
     * Change mode
     *
     * @param mode
     */
    public void setMode(int mode) {
        try {
            mRobotHeadService.setMode(mode);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setMode error", e);
        }
    }

    public AngularVelocity getYawAngularVelocity() {
        // TODO: 16/9/13
        AngularVelocity angularVelocity = new AngularVelocity(System.currentTimeMillis(), 0);
        return angularVelocity;
    }

    /**
     * Max velocity is xxx
     */
    public void setYawAngularVelocity(float yawAngularVelocity) {
        try {
            mRobotHeadService.setYawAngularVelocity(yawAngularVelocity);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setYawAngularVelocity error", e);
        }
    }

    public AngularVelocity getPitchAngularVelocity() {
        // TODO: 16/9/13
        AngularVelocity angularVelocity = new AngularVelocity(System.currentTimeMillis(), 0);
        return angularVelocity;
    }

    public void setPitchAngularVelocity(float pitchAngularVelocity) {
        try {
            mRobotHeadService.setPitchAngularVelocity(pitchAngularVelocity);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setPitchAngularVelocity error", e);
        }
    }

    public AngularVelocity getRollAngularVelocity() {
        // TODO: 16/9/13
        AngularVelocity angularVelocity = new AngularVelocity(System.currentTimeMillis(), 0);
        return angularVelocity;
    }

    /**
     * Get angle respect to robot base
     * Used only in MODE_SMOOTH_TACKING
     */
    public Angle getYawRespectBase() {
        try {
            return mRobotHeadService.getYawRespectBase();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getYawRespectBase error", e);
        }
    }

    public Angle getPitchRespectBase() {
        try {
            return mRobotHeadService.getPitchRespectBase();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getPitchRespectBase error", e);
        }
    }

    public Angle getRollRespectBase() {
        try {
            return mRobotHeadService.getRollRespectBase();
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("getRollRespectBase error", e);
        }
    }

    /**
     * Set yaw respect to robot base
     * Used only in MODE_SMOOTH_TACKING
     */
    public void setYawRespectBase(float yaw) {
        try {
            mRobotHeadService.setYawRespectBase(yaw);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setYawRespectBase error", e);
        }
    }

    /*
     * Set incremental value to pitch on current position
     */
    public void setIncrementalPitch(float pitch) {
        try {
            mRobotHeadService.setIncrementalPitch(pitch);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setIncrementalPitch error", e);
        }
    }

    public void setHeadLightMode(int mode) {
        try {
            mRobotHeadService.setHeadLeftLedMode(mode);
            mRobotHeadService.setHeadRightLedMode(mode);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setHeadLightMode error", e);
        }
    }

    public void setJointPosition(float yaw, float pitch) {
        try {
            mRobotHeadService.setJointPosition(yaw, pitch);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setJointPosition error", e);
        }
    }

    public void setJointYaw(float yaw) {
        try {
            mRobotHeadService.setJointYaw(yaw);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setJointYaw error", e);
        }
    }

    public void setJointPitch(float pitch) {
        try {
            mRobotHeadService.setJointPitch(pitch);
        } catch (RemoteException e) {
            e.printStackTrace();
            throw new RobotHeadException("setJointPitch error", e);
        }
    }

    private void bindError(Exception e) {
        mBindStateListener.onUnbind("bind error");
        Logger.e(TAG, "bind error", e);
        isBind = false;
        mContext.unbindService(mServiceConnection);
    }
}

