package com.autom8ed.lr2

import android.graphics.Bitmap
import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.vision.Vision;
import com.segway.robot.sdk.vision.calibration.ColorDepthCalibration
import com.segway.robot.sdk.vision.calibration.MotionModuleCalibration
import com.segway.robot.sdk.vision.frame.Frame
import com.segway.robot.sdk.vision.stream.StreamInfo
import com.segway.robot.sdk.vision.stream.StreamType
import kotlinx.coroutines.delay
import org.ros2.rcljava.publisher.Publisher
import sensor_msgs.msg.Image
import java.nio.ByteBuffer
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
        override fun nativeEncoding(): String {
            return "rgb8"
        }

        override fun copyFrameToRosImage(frame: Frame, msg: Image) {

        }
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
        override fun nativeEncoding(): String {
            return "16UC1"
        }

        override fun copyFrameToRosImage(frame: Frame, msg: Image) {

        }
    },
    FISH_EYE {
        private var mMotionModuleCalibration: MotionModuleCalibration? = null
        private var mRosCalibration: RosCameraCalibration? = null
        private var mBitmap: Bitmap? = null
        private var mStreamInfo: StreamInfo? = null

        override fun initCache(vision: Vision) {
            mMotionModuleCalibration = vision.motionModuleCalibrationData
            mStreamInfo = vision.getStreamInfo(StreamType.FISH_EYE)
            mBitmap = Bitmap.createBitmap(mStreamInfo!!.width, mStreamInfo!!.height, Bitmap.Config.ALPHA_8)
            Log.i("FISH_EYE", "Stream Info: ${mStreamInfo!!.width}x${mStreamInfo!!.height}")
            Log.i("FISH_EYE", "Bitmap size: ${mBitmap!!.byteCount}")
        }

        override fun copyFrameToRosImage(frame: Frame, msg: Image) {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)
            var bb: ByteBuffer = ByteBuffer.allocate(mBitmap!!.byteCount)
            mBitmap!!.copyPixelsToBuffer(bb)
            msg.data = bb.array().asList()

            msg.header.frameId = "loomo_fisheye"
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
        override fun nativeEncoding(): String {
            return "8UC1"
        }
    };

    abstract fun streamType(): Int
    abstract fun cameraFrameTopic(): String

    abstract fun nativeEncoding(): String

    abstract fun getRosCameraCalibration(): RosCameraCalibration

    abstract fun initCache(vision: Vision)

    abstract fun copyFrameToRosImage(frame: Frame, msg: sensor_msgs.msg.Image)
}

class VisionInterface constructor(ctx: android.content.Context, node: RosNode) {

    private class CameraPublisherContext constructor(vision: Vision, camera: Camera, topic: String, node: RosNode) : Vision.FrameListener {
        private val mVision: Vision = vision
        private val mCamera: Camera = camera
        private val mNode: RosNode = node
        private val mTopic: String = topic
        private val mFramePublisher: Publisher<sensor_msgs.msg.Image>
        private val mCompressedFramePublisher: Publisher<sensor_msgs.msg.CompressedImage>
        private val mInfoPublisher: Publisher<sensor_msgs.msg.CameraInfo>
        private val TAG: String = "CamPub_" + mCamera.name

        init {
            Log.i(TAG, "Created Camera Publisher for Camera " + mCamera.name);
            val info: StreamInfo = mVision.getStreamInfo(mCamera.streamType())
            Log.i(TAG, "StreamInfo: $info");
            // TODO: need to support image_depth
            mFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.Image::class.java,
                "$mTopic/image_color"
            )
            mCompressedFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.CompressedImage::class.java, "$mTopic/image_compressed")
            mInfoPublisher = mNode.node.createPublisher(sensor_msgs.msg.CameraInfo::class.java,
                "$mTopic/camera_info"
            )
        }

        override fun onNewFrame(streamType: Int, frame: Frame?) {
            if (frame == null) {
                return
            }

            // Convert the frame to a ROS format

            // Publish it via the node
            //mFramePublisher.publish("Got frame " + frame?.info.toString())

            Log.i(TAG, "Got new frame: $streamType");
            Log.i(TAG, "Resolution: ${frame.info.resolution}");
            Log.i(TAG, "IMU Timestamp: ${frame.info.imuTimeStamp}");
            Log.i(TAG, "Platform Timestamp: ${frame.info.platformTimeStamp}");

            val rawMsg: sensor_msgs.msg.Image = sensor_msgs.msg.Image()
            rawMsg.setEncoding(mCamera.nativeEncoding())
            mCamera.copyFrameToRosImage(frame, rawMsg)

            mFramePublisher.publish(rawMsg)
            // var compressed_msg: sensor_msgs.msg.CompressedImage = sensor_msgs.msg.CompressedImage()
        }

        fun startPublishing() {
            mCamera.initCache(mVision)
            mVision.startListenFrame(mCamera.streamType(), this);
        }

        fun stopPublishing() {
            mVision.stopListenFrame(mCamera.streamType())
        }
    }

    private var mVision: Vision = Vision.getInstance();
    private var mBindVisionListener: ServiceBinder.BindStateListener;
    private var mPublishers: EnumMap<Camera, CameraPublisherContext> =
        EnumMap(Camera::class.java);

    val TAG: String = "VisionIface"
    private var mNode: RosNode = node

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
        //CoroutineScope(Dispatchers.IO).launch {
        //    startCameraStream(Camera.FISH_EYE, "fisheye")
        //}
    }

    // On Suspend: unbind the service

    // On Resume: re-bind the service

    // Start a new camera stream and publish to the provided ROS topic
    suspend fun startCameraStream(camera: Camera, topic: String) {
        // Raise exception if we already have a publisher for this camera type
        if (mPublishers.containsKey(camera)) {
            throw IllegalStateException("Already have a publisher for this camera type")
        }

        // Vision interface must be started before we can start publishing
        // TODO: this is mega nasty. FIX THIS with some kind of threading once I'm
        // more familiar with Kotlin
        while (!mVision.isBind) {
            delay(100);
        }

        mPublishers[camera] = CameraPublisherContext(mVision, camera, topic, mNode)
        mPublishers[camera]!!.startPublishing()
    }

    public fun stopCameraStream(camera: Camera) {
        if (!mPublishers.containsKey(camera)) {
            throw IllegalStateException("No publisher for this camera type!")
        }

        mPublishers[camera]?.stopPublishing()
        mPublishers.remove(camera)
    }
}