package com.autom8ed.lr2.vision

import com.autom8ed.lr2.TfPublisherConstants
import com.segway.robot.sdk.vision.Vision
import com.segway.robot.sdk.vision.calibration.ColorDepthCalibration
import com.segway.robot.sdk.vision.calibration.MotionModuleCalibration
import com.segway.robot.sdk.vision.stream.StreamInfo
import com.segway.robot.sdk.vision.stream.StreamType

class Resolution(width: Int, height: Int, pixelBytes: Int) {
    public val mWidth: Int = width
    public val mHeight: Int = height
    public val mPixelBytes: Int = pixelBytes
}

abstract class LoomoCamera(vision: Vision, loomoStreamType: Int, imageType: ImageType, tfOpticalFrameId: String) {
    protected val mVision: Vision = vision
    protected val mStreamInfo: StreamInfo
    protected val mResolution: Resolution
    protected val mLoomoStreamType: Int = loomoStreamType
    protected val mImageType: ImageType = imageType
    protected val mTfOpticalFrameId: String = tfOpticalFrameId

    init {
        if (!mVision.isBind) {
            throw RuntimeException("mVision is not bound when camera is initialized")
        }

        mStreamInfo = mVision.getStreamInfo(loomoStreamType)
        mResolution = Resolution(mStreamInfo.width, mStreamInfo.height,
            mStreamInfo.pixelBytes.toInt()
        )
    }

    fun getImageType(): ImageType {
        return mImageType
    }

    fun getResolution(): Resolution {
        return mResolution
    }

    fun getLoomoStreamType(): Int {
        return mLoomoStreamType
    }

    fun getTfOpticalFrameId(): String {
        return mTfOpticalFrameId
    }

    fun startStream(listener: Vision.FrameListener) {
        mVision.startListenFrame(mLoomoStreamType, listener)
    }

    fun stopStream() {
        mVision.stopListenFrame(mLoomoStreamType)
    }

    protected fun getCameraInfoCore(): sensor_msgs.msg.CameraInfo {
        val msg: sensor_msgs.msg.CameraInfo = sensor_msgs.msg.CameraInfo()

        msg.header.frameId = getTfOpticalFrameId()
        msg.height = mStreamInfo.height
        msg.width = mStreamInfo.width

        return msg
    }

    abstract fun getCameraInfo(): sensor_msgs.msg.CameraInfo
}

class RealsenseDepthCamera(vision: Vision) : LoomoCamera(vision, StreamType.DEPTH, ImageType.DEPTH, TfPublisherConstants.REALSENSE_DEPTH_OPTICAL_FRAME_ID) {

    private var mColorDepthCalibration: ColorDepthCalibration = mVision.colorDepthCalibrationData
    private var mCameraInfo: sensor_msgs.msg.CameraInfo

    init {
        mCameraInfo = populateCameraInfo()
    }

    override fun getCameraInfo(): sensor_msgs.msg.CameraInfo {
        return mCameraInfo
    }

    private fun populateCameraInfo(): sensor_msgs.msg.CameraInfo {
        val msg: sensor_msgs.msg.CameraInfo = getCameraInfoCore()

        val intrinsic = mColorDepthCalibration.colorIntrinsic

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
        val p = DoubleArray(12) { 0.0 }
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
        msg.distortionModel = "plumb_bob"

        /*msg.header.stamp.sec =
            TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt()
        msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000)*/

        return msg
    }
}

class RealsenseColorCamera(vision: Vision) : LoomoCamera(vision, StreamType.COLOR, ImageType.COLOR, TfPublisherConstants.REALSENSE_COLOR_OPTICAL_FRAME_ID) {
    private var mColorDepthCalibration: ColorDepthCalibration = mVision.colorDepthCalibrationData
    private var mCameraInfo: sensor_msgs.msg.CameraInfo

    init {
        mCameraInfo = populateCameraInfo()
    }

    override fun getCameraInfo(): sensor_msgs.msg.CameraInfo {
        return mCameraInfo
    }

    private fun populateCameraInfo(): sensor_msgs.msg.CameraInfo {
        val msg: sensor_msgs.msg.CameraInfo = getCameraInfoCore()

        val intrinsic = mColorDepthCalibration.depthIntrinsic

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
        val p = DoubleArray(12) { 0.0 }
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
        msg.distortionModel = "plumb_bob"

        return msg
    }
}

class FisheyeCamera(vision: Vision) : LoomoCamera(vision, StreamType.FISH_EYE, ImageType.MONO, TfPublisherConstants.FISHEYE_OPTICAL_FRAME_ID) {
    private var mMotionModuleCalibration: MotionModuleCalibration = mVision.motionModuleCalibrationData
    private var mCameraInfo: sensor_msgs.msg.CameraInfo

    init {
        mCameraInfo = populateCameraInfo()
    }

    override fun getCameraInfo(): sensor_msgs.msg.CameraInfo {
        return mCameraInfo
    }

    private fun populateCameraInfo(): sensor_msgs.msg.CameraInfo {
        val msg: sensor_msgs.msg.CameraInfo = getCameraInfoCore()

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

        return msg
    }
}