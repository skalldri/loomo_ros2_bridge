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
import org.ros2.rcljava.qos.QoSProfile
import sensor_msgs.msg.CompressedImage
import sensor_msgs.msg.Image
import java.io.ByteArrayOutputStream
import java.nio.ByteBuffer
import java.nio.channels.Channels
import java.nio.channels.WritableByteChannel
import java.util.EnumMap
import java.util.concurrent.TimeUnit


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

val IMAGE_TOPIC_SUFFIX_MONO = "_raw"
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
            // Loomo delivers frames via callback as ARGB_8888
            mBitmap = Bitmap.createBitmap(mStreamInfo!!.width, mStreamInfo!!.height, Bitmap.Config.ARGB_8888)
            mByteBuffer = ByteBuffer.allocate(mBitmap!!.byteCount)
        }

        override fun streamType(): Int { return StreamType.COLOR }
        override fun cameraFrameTopicNamespace(): String { return "$REALSENSE_TOPIC/$REALSENSE_COLOR_NAMESPACE" }
        override fun cameraFrameTopicSuffix(): String { return IMAGE_TOPIC_SUFFIX_COLOR }
        override fun nativeEncoding(): String { return IMAGE_ENCODING_COLOR }

        override fun fillRosMessage(frame: Frame, msg: Image, timeSync: TimeSync): Boolean {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.position(0)

            // Copy from Bitmap -> Buffer
            // We ideally want to only copy the RGB elements and skip the Alpha channel...
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array()

            msg.header.frameId = TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()
            // msg.header.stamp = timeSync.getRosTime(frame.timestamp, frame.tid)

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo, timeSync: TimeSync): Boolean {
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
            val k =    DoubleArray(9) { 0.0 }
            k[0] = intrinsic.focalLength.x.toDouble()
            k[4] = intrinsic.focalLength.y.toDouble()
            k[2] = intrinsic.principal.x.toDouble()
            k[5] = intrinsic.principal.y.toDouble()
            k[8] = 1.0
            msg.k = k

            // For depth processing, the P matrix is the most important to publish
            // Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            //
            // For monocular cameras, Tx = Ty = 0
            //
            var p = DoubleArray(12) { 0.0 }
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
            msg.p = p

            val d = DoubleArray(5) { 0.0 }

            // Depth camera has valid distortion coefficients
            for (i in 0..4) {
                d[i] = intrinsic.distortion.value[i].toDouble()
            }

            msg.d = d

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: CompressedImage, timeSync: TimeSync): Boolean {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            val compressedOutStream: ByteArrayOutputStream = ByteArrayOutputStream();
            mBitmap!!.compress(Bitmap.CompressFormat.JPEG, 80, compressedOutStream)

            // Assign to ROS msg
            msg.data = compressedOutStream.toByteArray()

            msg.header.frameId = TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID
            msg.format = "jpeg"

            // msg.header.stamp = timeSync.getRosTime(frame.timestamp, frame.tid)

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
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

        override fun fillRosMessage(frame: Frame, msg: Image, timeSync: TimeSync): Boolean {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.clear()

            // Copy from Bitmap -> Buffer
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array()

            msg.header.frameId = TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()
            // msg.header.stamp = timeSync.getRosTime(frame.timestamp, frame.tid)

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo, timeSync: TimeSync): Boolean {
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
            val k = DoubleArray(9) { 0.0 }
            k[0] = intrinsic.focalLength.x.toDouble()
            k[4] = intrinsic.focalLength.y.toDouble()
            k[2] = intrinsic.principal.x.toDouble()
            k[5] = intrinsic.principal.y.toDouble()
            k[8] = 1.0
            msg.k = k

            // For depth processing, the P matrix is the most important to publish
            // Projection/camera matrix
            //     [fx'  0  cx' Tx]
            // P = [ 0  fy' cy' Ty]
            //     [ 0   0   1   0]
            //
            // For monocular cameras, Tx = Ty = 0
            //
            var p = DoubleArray(12) { 0.0 }
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
            msg.p = p

            val d = DoubleArray(5) { 0.0 }

            // Depth camera has valid distortion coefficients
            for (i in 0..4) {
                d[i] = intrinsic.distortion.value[i].toDouble()
            }

            msg.d = d

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: CompressedImage, timeSync: TimeSync): Boolean {
            return false
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

        override fun fillRosMessage(frame: Frame, msg: Image, timeSync: TimeSync): Boolean {
            // Loomo SDK can only copy images into an android.Bitmap. Do that first, then
            // extract the data. This is expensive... but what can you do.
            // TODO: figure out how to avoid this double-copy
            mBitmap!!.copyPixelsFromBuffer(frame.byteBuffer)

            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer!!.clear()

            // Copy from Bitmap -> Buffer
            mBitmap!!.copyPixelsToBuffer(mByteBuffer!!)

            // Assign to ROS msg
            msg.data = mByteBuffer!!.array()

            msg.header.frameId = TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID
            msg.encoding = this.nativeEncoding()
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width
            msg.step = mStreamInfo!!.width * mStreamInfo!!.pixelBytes.toInt()

            // msg.header.stamp = timeSync.getRosTime(frame.timestamp, frame.tid)
            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo, timeSync: TimeSync): Boolean {
            msg.header.frameId = TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID
            msg.height = mStreamInfo!!.height
            msg.width = mStreamInfo!!.width

            /*

            **** Calibrating ****
           mono pinhole calibration...
            D = [-0.20999224085010745, 0.03005997134956105, 0.00028780071510052047, 6.403100484305247e-05, 0.0]
            K = [278.0376154285591, 0.0, 315.4365678931718, 0.0, 276.6463084317693, 237.7256909765486, 0.0, 0.0, 1.0]
            R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            P = [190.4833526611328, 0.0, 315.73524543965686, 0.0, 0.0, 218.78814697265625, 236.13651827145077, 0.0, 0.0, 0.0, 1.0, 0.0]
            */

            // Intrinsic camera matrix for the raw (distorted) images.
            //     [fx  0 cx]
            // K = [ 0 fy cy]
            //     [ 0  0  1]
            // Stored as a flat array of 9 float64 (doubles)
            // Row-major order
            val k: DoubleArray = doubleArrayOf(278.0376154285591, 0.0, 315.4365678931718, 0.0, 276.6463084317693, 237.7256909765486, 0.0, 0.0, 1.0)

            // Loomo calibration is busted, manually calibrated
            /*for (row: Int in 0..2) {
                for (col: Int in 0..2) {
                    k[(row * 3) + col] = mMotionModuleCalibration!!.fishEyeIntrinsics.kf.matrix[row][col].toDouble()
                }
            }*/

            msg.k = k
            val d: DoubleArray = doubleArrayOf(-0.20999224085010745, 0.03005997134956105, 0.00028780071510052047, 6.403100484305247e-05, 0.0)
            // d[0] = mMotionModuleCalibration!!.fishEyeIntrinsics.distortion.value[0].toDouble()
            // TODO: My Loomo has NaN for the other 4 distortion parameters...

            msg.d = d

            val p: DoubleArray = doubleArrayOf(190.4833526611328, 0.0, 315.73524543965686, 0.0, 0.0, 218.78814697265625, 236.13651827145077, 0.0, 0.0, 0.0, 1.0, 0.0)
            msg.p = p

            msg.distortionModel = "plumb_bob"

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            return true
        }

        override fun fillRosMessage(frame: Frame, msg: CompressedImage, timeSync: TimeSync): Boolean {
            return false
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

    abstract fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.Image, timeSync: TimeSync): Boolean

    abstract fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CameraInfo, timeSync: TimeSync): Boolean

    abstract fun fillRosMessage(frame: Frame, msg: sensor_msgs.msg.CompressedImage, timeSync: TimeSync): Boolean
}

class VisionInterface (ctx: android.content.Context, node: RosNode, tfPublisher: TfPublisher, timeSync: TimeSync) {

    private class CameraPublisherContext (vision: Vision, camera: Camera, topic: String, node: RosNode, tfPublisher: TfPublisher, timeSync: TimeSync) : Vision.FrameListener {
        private val mVision: Vision = vision
        private val mCamera: Camera = camera
        private val mNode: RosNode = node
        private val mTopic: String = topic
        private val mFramePublisher: Publisher<sensor_msgs.msg.Image>
        private val mCompressedFramePublisher: Publisher<sensor_msgs.msg.CompressedImage>
        private val mCameraInfoPublisher: Publisher<sensor_msgs.msg.CameraInfo>
        private val mTfPublisher: TfPublisher = tfPublisher
        private val mTimeSync: TimeSync = timeSync
        private val TAG: String = "CamPub_" + mCamera.name

        // Perf counters
        var mPerfImage = PerfCounter("${mCamera.cameraFrameTopicNamespace()}/it")
        var mPerfImagePublish = PerfCounter("${mCamera.cameraFrameTopicNamespace()}/ip")
        var mPerfCamInfo = PerfCounter("${mCamera.cameraFrameTopicNamespace()}/ct")
        var mPerfCamInfoPublish = PerfCounter("${mCamera.cameraFrameTopicNamespace()}/cp")

        init {
            val rawFrameTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/image${mCamera.cameraFrameTopicSuffix()}"
            mFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.Image::class.java,
                rawFrameTopic,
                QoSProfile.SENSOR_DATA
            )
            Log.i(TAG, "${mCamera.name}: created topic $rawFrameTopic")

            val compressedFrameTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/image${mCamera.cameraFrameTopicSuffix()}/compressed"
            // TODO: fix image_transport to accept any kind of QoSProfile
            mCompressedFramePublisher = mNode.node.createPublisher(sensor_msgs.msg.CompressedImage::class.java, compressedFrameTopic, QoSProfile.SENSOR_DATA)
            Log.i(TAG, "${mCamera.name}: created topic $compressedFrameTopic")

            val cameraInfoTopic = "$mTopic/${mCamera.cameraFrameTopicNamespace()}/image${mCamera.cameraFrameTopicSuffix()}/$CAMERA_INFO_TOPIC"
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
                mTfPublisher.indicateTfNeededAtTime(mTfPublisher.captureTfContext(frame.info.platformTimeStamp))
            }

            if (mCamera == Camera.DEPTH || mCamera == Camera.FISH_EYE) {
                // mPerfImage.start()
                val rawMsg: sensor_msgs.msg.Image = sensor_msgs.msg.Image()
                mCamera.fillRosMessage(frame, rawMsg, mTimeSync)
                // mPerfImage.stop()

                //mPerfImagePublish.start()
                mFramePublisher.publish(rawMsg)
                //mPerfImagePublish.stop()
            } else if (mCamera == Camera.COLOR) {
                // Color camera publishes compressed frames
                //mPerfImage.start()
                val rawMsg: sensor_msgs.msg.CompressedImage = sensor_msgs.msg.CompressedImage()
                mCamera.fillRosMessage(frame, rawMsg, mTimeSync)
                //mPerfImage.stop()

                //mPerfImagePublish.start()
                mCompressedFramePublisher.publish(rawMsg)
                //mPerfImagePublish.stop()
            } else {
                assert(false);
            }

            //mPerfCamInfo.start()
            val cameraInfoMsg: sensor_msgs.msg.CameraInfo = sensor_msgs.msg.CameraInfo()
            val publishCameraInfo = mCamera.fillRosMessage(frame, cameraInfoMsg, mTimeSync)
            //mPerfCamInfo.stop()

            //mPerfCamInfoPublish.start()
            if (publishCameraInfo) {
                mCameraInfoPublisher.publish(cameraInfoMsg)
            }
            //mPerfCamInfoPublish.stop()
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
    private val mTimeSync: TimeSync = timeSync

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

        mPublishers[camera] = CameraPublisherContext(mVision, camera, topic, mNode, mTfPublisher, mTimeSync)
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