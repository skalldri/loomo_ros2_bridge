package com.autom8ed.lr2.vision

import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile
import org.ros2.rcljava.qos.policies.Reliability

/**
 * RELIABLE with a keep-last history of [depth] samples. Built fresh for every publisher: the
 * QoSProfile setters mutate in place, so the shared QoSProfile.SENSOR_DATA/DEFAULT statics must
 * never be modified.
 *
 * Why not best effort: with asynchronous publishing a sample waits in the writer history until
 * Fast DDS's sender thread transmits it, and the SENSOR_DATA profile keeps only 5. Whenever the
 * USB Ethernet adapter stalled the sender for a few frames (200-330 ms gaps on every topic at
 * once on 2026-10-05), the older samples were evicted unsent: 151 of 1 788 colour camera_info
 * messages in one minute. Fast DDS KEEP_LAST still evicts when the history is full, so the depth
 * must cover the longest stall of the sender or of the slowest reader; RELIABLE adds the
 * retransmission of fragments lost on the wire.
 */
fun reliableKeepLast(depth: Int): QoSProfile =
    QoSProfile.keepLast(depth).setReliability(Reliability.RELIABLE)

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
    stats: StreamStats
) {
    private val mNode: RosNode = node
    val stats: StreamStats = stats
    private val mCamera: LoomoCamera = camera
    private val mBaseImageTopic: String = baseImageTopic
    private val mCameraInfoTopic: String = getCameraInfoTopic(mBaseImageTopic)
    private val mCompressedImageTopic: String = getCompressedTopic(mBaseImageTopic)
    private val mH264ImageTopic: String = getH264Topic(mBaseImageTopic)

    // These topics are always available. History depths: one second of frames for the raw
    // image and its camera info, two seconds for H.264 (frames are 15-85 KB; the Jetson's decoder
    // has shown 250 ms stalls).
    private val mBaseImagePublisher: BaseImagePublisher =
        BaseImagePublisher(mNode, mBaseImageTopic, mCamera, reliableKeepLast(RAW_HISTORY))
    private val mCameraInfoPublisher: CameraInfoPublisher =
        CameraInfoPublisher(mNode, mCameraInfoTopic, mCamera, reliableKeepLast(RAW_HISTORY))

    // These topics are published based on the image type
    private var mCompressedFramePublisher: CompressedImagePublisher? = null
    private var mH264FramePublisher: H264ImagePublisher? = null

    private val TAG = "ImageTransport - $mBaseImageTopic"
    private val mFramePerf: PerfCounter = PerfCounter("ImageTransport - $mBaseImageTopic - frame")

    init {
        // Compressed publishers
        if (mCamera.getImageType().supportsCompressedPublisher()) {
            mCompressedFramePublisher =
                CompressedImagePublisher(mNode, mCompressedImageTopic, mCamera, reliableKeepLast(RAW_HISTORY))
        }

        if (mCamera.getImageType().supportsH264Publisher()) {
            mH264FramePublisher = H264ImagePublisher(mNode, mH264ImageTopic, mCamera, reliableKeepLast(H264_HISTORY))
        }

        mBaseImagePublisher.stats = stats
        mCameraInfoPublisher.stats = stats
        mCompressedFramePublisher?.stats = stats
        mH264FramePublisher?.stats = stats

        MessageWarmup.warm()
    }

    companion object {
        const val RAW_HISTORY = 30
        const val H264_HISTORY = 60
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