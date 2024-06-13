package com.autom8ed.lr2

import android.graphics.Bitmap
import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.vision.Vision
import com.segway.robot.sdk.vision.calibration.ColorDepthCalibration
import com.segway.robot.sdk.vision.calibration.MotionModuleCalibration
import com.segway.robot.sdk.vision.frame.Frame
import com.segway.robot.sdk.vision.stream.StreamInfo
import com.segway.robot.sdk.vision.stream.StreamType
import kotlinx.coroutines.delay
import org.ros2.rcljava.publisher.Publisher
import sensor_msgs.msg.Image
import java.nio.ByteBuffer
import java.nio.channels.Channels
import java.nio.channels.WritableByteChannel
import java.util.EnumMap


val CAMERA_INFO_TOPIC = "camera_info"

val REALSENSE_TOPIC = "realsense"
val REALSENSE_DEPTH_NAMESPACE = "depth"
val REALSENSE_COLOR_NAMESPACE = "color"

val FISHEYE_TOPIC = "fisheye"

// Per ROS CameraInfo topic specification:
// image: monochrome, distorted
// image_rect: monochrome, rectified
// image_color: color, distorted
// image_color_rect: color, rectified
//
// By convention of the Realsense driver, publish the
// depth image as a monochrome or raw image

val IMAGE_TOPIC_SUFFIX_COLOR = "_color"
val IMAGE_ENCODING_COLOR = "rgba8"

val IMAGE_TOPIC_SUFFIX_MONO = ""
val IMAGE_ENCODING_MONO = "mono8"

val IMAGE_TOPIC_SUFFIX_DEPTH = "_depth"
val IMAGE_ENCODING_DEPTH = "16UC1"

enum class Camera {
    COLOR {
        private var mColorDepthCalibration: ColorDepthCalibration? = null
        private var mStreamInfo: StreamInfo? = null
        private var mBitmap: Bitmap? = null
        private var mByteBuffer: ByteBuffer? = null

        override fun initCache(vision: Vision) {
            mColorDepthCalibration = vision.colorDepthCalibrationData
            mStreamInfo = vision.getStreamInfo(StreamType.COLOR)
            mBitmap = Bitmap.createBitmap(mStreamInfo!!.width, mStreamInfo!!.height, Bitmap.Config.ARGB_8888)
            mByteBuffer = ByteBuffer.allocate(mBitmap!!.byteCount)
        }

        override fun streamType(): Int { return StreamType.COLOR }
        override fun cameraFrameTopicNamespace(): String { return "$REALSENSE_TOPIC/$REALSENSE_COLOR_NAMESPACE" }
        override fun cameraFrameTopicSuffix(): String { return IMAGE_TOPIC_SUFFIX_COLOR }
        override fun nativeEncoding(): String { return IMAGE_ENCODING_COLOR }

        override fun fillRosMessage(frame: Frame, msg: Image) {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.position(0)

            // Copy from Bitmap -> Buffer
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array().asList()

            msg.header.frameId = TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo) {
            msg.header.frameId = TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width

            val intrinsic = mColorDepthCalibration!!.colorIntrinsic

            // Intrinsic camera matrix for the raw (distorted) images.
            //     [fx  0 cx]
            // K = [ 0 fy cy]
            //     [ 0  0  1]
            // Stored as a flat array of 9 float64 (doubles)
            // Row-major order
            val k = Array<Double>(9) { 0.0 }
            k[0] = intrinsic.focalLength.x.toDouble()
            k[4] = intrinsic.focalLength.y.toDouble()
            k[2] = intrinsic.principal.x.toDouble()
            k[5] = intrinsic.principal.y.toDouble()
            k[8] = 1.0
            msg.k = k.asList()

            // For depth processing, the P matrix is the most important to publish
            // Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            //
            // For monocular cameras, Tx = Ty = 0
            //
            var p = Array<Double>(12) { 0.0 }
            // fx'
            p[0] = intrinsic.focalLength.x.toDouble()
            // cx'
            p[2] = intrinsic.principal.x.toDouble()
            // Tx
            p[3] = 0.0
            // fy'
            p[5] = intrinsic.focalLength.y.toDouble()
            // cy'
            p[6] = intrinsic.principal.y.toDouble()
            // Ty
            p[7] = 0.0
            p[10] = 1.0
            msg.p = p.asList()

            val d = Array<Double>(5) { 0.0 }

            // Depth camera has valid distortion coefficients
            for (i in 0..4) {
                d[i] = intrinsic.distortion.value[i].toDouble()
            }

            msg.d = d.asList()
        }
    },
    DEPTH {
        private var mColorDepthCalibration: ColorDepthCalibration? = null
        private var mStreamInfo: StreamInfo? = null
        private var mBitmap: Bitmap? = null
        private var mByteBuffer: ByteBuffer? = null

        override fun initCache(vision: Vision) {
            mColorDepthCalibration = vision.colorDepthCalibrationData
            mStreamInfo = vision.getStreamInfo(StreamType.DEPTH)
            mBitmap = Bitmap.createBitmap(mStreamInfo!!.width, mStreamInfo!!.height, Bitmap.Config.RGB_565)
            mByteBuffer = ByteBuffer.allocate(mBitmap!!.byteCount)
        }

        override fun streamType(): Int { return StreamType.DEPTH }
        override fun cameraFrameTopicNamespace(): String { return "$REALSENSE_TOPIC/$REALSENSE_DEPTH_NAMESPACE" }
        override fun cameraFrameTopicSuffix(): String { return IMAGE_TOPIC_SUFFIX_DEPTH }
        override fun nativeEncoding(): String { return IMAGE_ENCODING_DEPTH }

        override fun fillRosMessage(frame: Frame, msg: Image) {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.clear()

            // Copy from Bitmap -> Buffer
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array().asList()

            msg.header.frameId = TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo) {
            msg.header.frameId = TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width

            val intrinsic = mColorDepthCalibration!!.depthIntrinsic

            // Intrinsic camera matrix for the raw (distorted) images.
            //     [fx  0 cx]
            // K = [ 0 fy cy]
            //     [ 0  0  1]
            // Stored as a flat array of 9 float64 (doubles)
            // Row-major order
            val k = Array<Double>(9) { 0.0 }
            k[0] = intrinsic.focalLength.x.toDouble()
            k[4] = intrinsic.focalLength.y.toDouble()
            k[2] = intrinsic.principal.x.toDouble()
            k[5] = intrinsic.principal.y.toDouble()
            k[8] = 1.0
            msg.k = k.asList()

            // For depth processing, the P matrix is the most important to publish
            // Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            //
            // For monocular cameras, Tx = Ty = 0
            //
            var p = Array<Double>(12) { 0.0 }
            // fx'
            p[0] = intrinsic.focalLength.x.toDouble()
            // cx'
            p[2] = intrinsic.principal.x.toDouble()
            // Tx
            p[3] = 0.0
            // fy'
            p[5] = intrinsic.focalLength.y.toDouble()
            // cy'
            p[6] = intrinsic.principal.y.toDouble()
            // Ty
            p[7] = 0.0
            p[10] = 1.0
            msg.p = p.asList()

            val d = Array<Double>(5) { 0.0 }

            // Depth camera has valid distortion coefficients
            for (i in 0..4) {
                d[i] = intrinsic.distortion.value[i].toDouble()
            }

            msg.d = d.asList()
        }
    },
    FISH_EYE {
        private var mMotionModuleCalibration: MotionModuleCalibration? = null
        private var mStreamInfo: StreamInfo? = null
        private var mBitmap: Bitmap? = null
        private var mByteBuffer: ByteBuffer? = null

        override fun initCache(vision: Vision) {
            mMotionModuleCalibration = vision.motionModuleCalibrationData
            mStreamInfo = vision.getStreamInfo(StreamType.FISH_EYE)
            mBitmap = Bitmap.createBitmap(mStreamInfo!!.width, mStreamInfo!!.height, Bitmap.Config.ALPHA_8)
            mByteBuffer = ByteBuffer.allocate(mBitmap!!.byteCount)
        }

        override fun fillRosMessage(frame: Frame, msg: Image) {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.clear()

            // Copy from Bitmap -> Buffer
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array().asList()

            msg.header.frameId = TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo) {
            msg.header.frameId = TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width

            // Intrinsic camera matrix for the raw (distorted) images.
            //     [fx  0 cx]
            // K = [ 0 fy cy]
            //     [ 0  0  1]
            // Stored as a flat array of 9 float64 (doubles)
            // Row-major order
            var k = Array<Double>(9) { 0.0 }
            for (row: Int in 0..2) {
                for (col: Int in 0..2) {
                    k[(row * 3) + col] = mMotionModuleCalibration!!.fishEyeIntrinsics.kf.matrix[row][col].toDouble()
                }
            }

            msg.k = k.asList()

            var d = Array<Double>(5) { 0.0 }

            // d[0] = mMotionModuleCalibration!!.fishEyeIntrinsics.distortion.value[0].toDouble()
            // TODO: My Loomo has NaN for the other 4 distortion parameters...

            msg.d = d.asList()
            msg.distortionModel = "plumb_bob"
        }

        override fun streamType(): Int { return StreamType.FISH_EYE }
        override fun cameraFrameTopicNamespace(): String { return FISHEYE_TOPIC }
        override fun cameraFrameTopicSuffix(): String { return IMAGE_TOPIC_SUFFIX_MONO }
        override fun nativeEncoding(): String { return IMAGE_ENCODING_MONO }
    };

    abstract fun streamType(): Int

    /**
     * The topic-namespace for this camera.
     *
     * We will produce a topics in the format:
     * /loomo/<namespace>/camera_info
     * /loomo/<namespace>/image_<suffix>
     */
    abstract fun cameraFrameTopicNamespace(): String

    /**
     * The topic-suffix for this camera's frame output.
     *
     * We will produce a topics in the format:
     * /loomo/<namespace>/camera_info
     * /loomo/<namespace>/image_<suffix>
    */
    abstract fun cameraFrameTopicSuffix(): String

    abstract fun nativeEncoding(): String

    abstract fun initCache(vision: Vision)

    abstract fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.Image)

    abstract fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo)
}

class VisionInterface constructor(ctx: android.content.Context, node: RosNode, tfPublisher: TfPublisher) {

    private class CameraPublisherContext constructor(vision: Vision, camera: Camera, topic: String, node: RosNode, tfPublisher: TfPublisher) : Vision.FrameListener {
        private val mVision: Vision = vision
        private val mCamera: Camera = camera
        private val mNode: RosNode = node
        private val mTopic: String = topic
        private val mFramePublisher: Publisher<sensor_msgs.msg.Image>
        private val mCompressedFramePublisher: Publisher<sensor_msgs.msg.CompressedImage>
        private val mCameraInfoPublisher: Publisher<sensor_msgs.msg.CameraInfo>
        private val mTfPublisher: TfPublisher = tfPublisher
        private val TAG: String = "CamPub_" + mCamera.name

        init {
            val rawFrameTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/image${mCamera.cameraFrameTopicSuffix()}"
            mFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.Image::class.java,
                rawFrameTopic
            )
            Log.i(TAG, "${mCamera.name}: created topic $rawFrameTopic")

            val compressedFrameTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/image_compressed"
            mCompressedFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.CompressedImage::class.java, compressedFrameTopic)
            Log.i(TAG, "${mCamera.name}: created topic $compressedFrameTopic");

            val cameraInfoTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/$CAMERA_INFO_TOPIC"
            mCameraInfoPublisher = mNode.node.createPublisher(sensor_msgs.msg.CameraInfo::class.java,
                cameraInfoTopic
            )
            Log.i(TAG, "${mCamera.name}: created topic $cameraInfoTopic");
        }

        override fun onNewFrame(streamType: Int, frame: Frame?) {
            if (frame == null) {
                return
            }

            // Tell the TF publisher we need a transform at this time
            // TODO: can we submit other cameras?
            if (mCamera == Camera.DEPTH) {
                mTfPublisher.indicateTfNeededAtTime(frame.info.platformTimeStamp)
            }

            val rawMsg: sensor_msgs.msg.Image = sensor_msgs.msg.Image()
            mCamera.fillRosMessage(frame, rawMsg)
            mFramePublisher.publish(rawMsg)

            val cameraInfoMsg: sensor_msgs.msg.CameraInfo = sensor_msgs.msg.CameraInfo()
            mCamera.fillRosMessage(frame, cameraInfoMsg)
            mCameraInfoPublisher.publish(cameraInfoMsg)
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
    private val mTfPublisher: TfPublisher = tfPublisher

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

        mPublishers[camera] = CameraPublisherContext(mVision, camera, topic, mNode, mTfPublisher)
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