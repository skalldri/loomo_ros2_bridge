package com.autom8ed.lr2.vision

import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile

fun getCameraInfoTopic(baseImageTopic: String): String {
    var infoTopic: String = ""
    var tokens = baseImageTopic.split("/")

    if (tokens.isNotEmpty()) {
        // Drop the last token which should be the final component
        // of the topic path
        tokens = tokens.dropLast(1)

        for (t in tokens) {
            if (t.isEmpty()) {
                continue
            }

            infoTopic += "/"
            infoTopic += t
            print("Tok: '$t'\n")
        }
    }

    infoTopic += "/camera_info"
    return infoTopic
}

fun getCompressedTopic(baseImageTopic: String): String {
    return "$baseImageTopic/compressed"
}

fun getCompressedDepthTopic(baseImageTopic: String): String {
    return "$baseImageTopic/compressedDepth"
}

fun getH264Topic(baseImageTopic: String): String {
    return "$baseImageTopic/h264"
}


class ImageTransport(
    node: RosNode,
    baseImageTopic: String,
    camera: LoomoCamera,
    stats: StreamStats,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) {
    private val mNode: RosNode = node
    val stats: StreamStats = stats
    private val mQos: QoSProfile = qos
    private val mCamera: LoomoCamera = camera
    private val mBaseImageTopic: String = baseImageTopic
    private val mCameraInfoTopic: String = getCameraInfoTopic(mBaseImageTopic)
    private val mCompressedImageTopic: String = getCompressedTopic(mBaseImageTopic)
    private val mH264ImageTopic: String = getH264Topic(mBaseImageTopic)

    // These topics are always available
    private val mBaseImagePublisher: BaseImagePublisher =
        BaseImagePublisher(mNode, mBaseImageTopic, mCamera, mQos)
    private val mCameraInfoPublisher: CameraInfoPublisher =
        CameraInfoPublisher(mNode, mCameraInfoTopic, mCamera, mQos)

    // These topics are published based on the image type
    private var mCompressedFramePublisher: CompressedImagePublisher? = null
    private var mH264FramePublisher: H264ImagePublisher? = null

    private val TAG = "ImageTransport - $mBaseImageTopic"
    private val mFramePerf: PerfCounter = PerfCounter("ImageTransport - $mBaseImageTopic - frame")

    init {
        // Compressed publishers
        if (mCamera.getImageType().supportsCompressedPublisher()) {
            mCompressedFramePublisher =
                CompressedImagePublisher(mNode, mCompressedImageTopic, mCamera, mQos)
        }

        if (mCamera.getImageType().supportsH264Publisher()) {
            mH264FramePublisher = H264ImagePublisher(mNode, mH264ImageTopic, mCamera, mQos)
        }

        mBaseImagePublisher.stats = stats
        mCameraInfoPublisher.stats = stats
        mCompressedFramePublisher?.stats = stats
        mH264FramePublisher?.stats = stats
    }

    /** Runs on the stream's FrameWorker thread with a slot copied out of the SDK callback. */
    fun publish(slot: FrameSlot) {
        val platformTimeStamp = slot.platformTimeStampUs
        val frameNum = slot.frameNum

        mFramePerf.start()

        // Always publish the camera info
        mCameraInfoPublisher.publish(platformTimeStamp, frameNum)

        // The raw image straight from the slot's array: no Bitmap round trip
        mBaseImagePublisher.publish(slot, platformTimeStamp, frameNum)

        // Safe access: does not call if NULL
        mH264FramePublisher?.publish(slot, platformTimeStamp, frameNum)
        mFramePerf.stop()
    }
}