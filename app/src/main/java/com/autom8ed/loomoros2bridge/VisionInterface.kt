package com.autom8ed.loomoros2bridge

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.calibration.ColorDepthCalibration
import com.segway.robot.sdk.vision.calibration.Intrinsic
import com.segway.robot.sdk.vision.calibration.MotionModuleCalibration
import com.segway.robot.sdk.vision.frame.Frame
import com.segway.robot.sdk.vision.stream.StreamInfo
import com.segway.robot.sdk.vision.stream.StreamType
import kotlinx.coroutines.delay
import java.util.EnumMap

val CAMERA_INFO_TOPIC = "camera_info"
val RGB_TOPIC = "rgb"
val MONO_TOPIC = "mono"

class RosCameraCalibration {
    var k: Array<Float> = Array<Float>(3*3) { 0.0F }
    var d: Array<Float> = Array<Float>(5) { 0.0F }
    var p: Array<Float> = Array<Float>(3*4) { 0.0F }
}

enum class Camera {
    COLOR {
        private var mColorDepthCalibration: ColorDepthCalibration? = null
        private var mRosCalibration: RosCameraCalibration? = null

        override fun initCache(vision: Vision) {
            mColorDepthCalibration = vision.colorDepthCalibrationData
        }

        override fun getRosCameraCalibration(): RosCameraCalibration {
            if (mRosCalibration != null) {
                return mRosCalibration as RosCameraCalibration
            }

            if (mColorDepthCalibration == null) {
                throw IllegalStateException("No cached calibration to process!")
            }

            // First time we got called... calculate the camera calibration
            mRosCalibration = RosCameraCalibration()

            // TODO: consume mColorDepthCalibration to compute calibration

            return mRosCalibration as RosCameraCalibration
        }

        override fun streamType(): Int { return StreamType.COLOR }
        override fun cameraFrameTopic(): String { return RGB_TOPIC }
    },
    DEPTH {
        private var mColorDepthCalibration: ColorDepthCalibration? = null
        private var mRosCalibration: RosCameraCalibration? = null

        override fun initCache(vision: Vision) {
            mColorDepthCalibration = vision.colorDepthCalibrationData
        }

        override fun getRosCameraCalibration(): RosCameraCalibration {
            if (mRosCalibration != null) {
                return mRosCalibration as RosCameraCalibration
            }

            if (mColorDepthCalibration == null) {
                throw IllegalStateException("No cached calibration to process!")
            }

            // First time we got called... calculate the camera calibration
            mRosCalibration = RosCameraCalibration()

            // TODO: consume mColorDepthCalibration to compute calibration

            return mRosCalibration as RosCameraCalibration
        }

        override fun streamType(): Int { return StreamType.DEPTH }
        override fun cameraFrameTopic(): String { return MONO_TOPIC }
    },
    FISH_EYE {
        private var mMotionModuleCalibration: MotionModuleCalibration? = null
        private var mRosCalibration: RosCameraCalibration? = null

        override fun initCache(vision: Vision) {
            mMotionModuleCalibration = vision.motionModuleCalibrationData
        }

        override fun getRosCameraCalibration(): RosCameraCalibration {
            if (mRosCalibration != null) {
                return mRosCalibration as RosCameraCalibration
            }

            if (mMotionModuleCalibration == null) {
                throw IllegalStateException("No cached calibration to process!")
            }

            // First time we got called... calculate the camera calibration
            mRosCalibration = RosCameraCalibration()

            // TODO: consume mColorDepthCalibration to compute calibration

            return mRosCalibration as RosCameraCalibration
        }

        override fun streamType(): Int { return StreamType.FISH_EYE }
        override fun cameraFrameTopic(): String { return RGB_TOPIC }
    };

    abstract fun streamType(): Int
    abstract fun cameraFrameTopic(): String

    abstract fun getRosCameraCalibration(): RosCameraCalibration

    abstract fun initCache(vision: Vision)
}

class VisionInterface constructor(ctx: android.content.Context) {

    private class CameraPublisherContext constructor(vision: Vision, camera: Camera, topic: String) : Vision.FrameListener {
        private val mVision: Vision = vision;
        private val mCamera: Camera = camera;
        private val mTopic: String = topic;
        private var mFramePublisher: TopicPublisher<String> = TopicPublisher("$mTopic/${mCamera.cameraFrameTopic()}");
        private var mCameraInfoPublisher: TopicPublisher<String> = TopicPublisher("$mTopic/$CAMERA_INFO_TOPIC");
        private val TAG: String = "CamPub_" + mCamera.name

        init {
            Log.i(TAG, "Created Camera Publisher for Camera " + mCamera.name);
            val info: StreamInfo = mVision.getStreamInfo(mCamera.streamType())
            Log.i(TAG, "StreamInfo: $info");
        }

        override fun onNewFrame(streamType: Int, frame: Frame?) {
            mFramePublisher.publish("Got frame " + frame?.info.toString())
        }

        fun startPublishing() {
            mVision.startListenFrame(mCamera.streamType(), this);

            // TODO: setup topic publisher

            // TODO: publish camera_info with camera extrinsics/intrinsics\
            mCamera.initCache(mVision)
            mCameraInfoPublisher.publish("Camera Info: ")
        }

        fun stopPublishing() {
            mVision.stopListenFrame(mCamera.streamType())
        }
    }

    private var mVision: Vision = Vision.getInstance();
    private var mBindVisionListener: ServiceBinder.BindStateListener;
    private var publishers: EnumMap<Camera, CameraPublisherContext> =
        EnumMap(com.autom8ed.loomoros2bridge.Camera::class.java);

    val TAG: String = "VisionIface"

    init {
        mBindVisionListener = object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.i(TAG, "mBindVisionListener onBind() called");
            }

            override fun onUnbind(reason: String) {
                Log.i(TAG, "mBindVisionListener onUnbind() called with: reason = [$reason]");
            }
        }

        // Connect to the service
        if (!mVision.bindService(ctx, mBindVisionListener)) {
            throw IllegalStateException("Failed to bind to vision service")
        }

        // RealSense color images
        // startCameraStream(StreamType.COLOR)
        // RealSense depth images
        // startCameraStream(StreamType.DEPTH)
        // Fisheye camera: 640*480 (really? seems small...)
        // startCameraStream(StreamType.FISH_EYE)
    }

    // On Suspend: unbind the service

    // On Resume: re-bind the service

    // Start a new camera stream and publish to the provided ROS topic
    suspend fun startCameraStream(camera: Camera, topic: String) {
        // Raise exception if we already have a publisher for this camera type
        if (publishers.containsKey(camera)) {
            throw IllegalStateException("Already have a publisher for this camera type")
        }

        // Vision interface must be started before we can start publishing
        // TODO: this is mega nasty. FIX THIS with some kind of threading once I'm
        // more familiar with Kotlin
        while (!mVision.isBind) {
            delay(100);
        }

        publishers[camera] = CameraPublisherContext(mVision, camera, topic)
        publishers[camera]?.startPublishing()
    }

    public fun stopCameraStream(camera: Camera) {
        if (!publishers.containsKey(camera)) {
            throw IllegalStateException("No publisher for this camera type!")
        }

        publishers[camera]?.stopPublishing()
        publishers.remove(camera)
    }
}