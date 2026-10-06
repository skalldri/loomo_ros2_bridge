package com.autom8ed.lr2.vision

import android.util.Log
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.StatsReporter
import com.autom8ed.lr2.TfPublisher
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.vision.Vision
import com.segway.robot.sdk.vision.frame.Frame
import kotlinx.coroutines.delay
import java.util.concurrent.Semaphore

class CameraInterface(
    ctx: android.content.Context,
    node: RosNode,
    tfPublisher: TfPublisher,
    statsReporter: StatsReporter
) {

    private val TAG: String = "CameraInterface"
    private val mVision: Vision = Vision.getInstance()
    private val mVisionSem: Semaphore = Semaphore(1)

    private val mBindVisionListener: ServiceBinder.BindStateListener =
        object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.i(TAG, "mBindVisionListener onBind() called")
                mVisionSem.release()
            }

            override fun onUnbind(reason: String) {
                Log.i(TAG, "mBindVisionListener onUnbind() called with: reason = [$reason]")
            }
        }

    private val mNode: RosNode = node
    private val mTfPublisher: TfPublisher = tfPublisher
    private val mStatsReporter: StatsReporter = statsReporter
    private lateinit var mRealsenseDepthCamera: LoomoCamera
    private lateinit var mRealsenseColorCamera: LoomoCamera
    private lateinit var mFisheyeCamera: LoomoCamera
    private lateinit var mRealsenseDepthPublisher: ImageTransport
    private lateinit var mRealsenseColorPublisher: ImageTransport
    private lateinit var mFisheyePublisher: ImageTransport

    init {
        // Connect to the service
        if (!mVision.bindService(ctx, mBindVisionListener)) {
            throw IllegalStateException("Failed to bind to vision service")
        }
    }

    suspend fun start() {
        // JANK WARNING!
        while (!mVision.isBind) {
            delay(100);
        }

        // Setup the camera publishers, each with its own delivery accounting
        mRealsenseDepthCamera = RealsenseDepthCamera(mVision)
        mRealsenseDepthPublisher = ImageTransport(
            mNode,
            "/loomo/realsense/depth/image_depth_rect",
            mRealsenseDepthCamera,
            newStats("depth")
        )

        mRealsenseColorCamera = RealsenseColorCamera(mVision)
        mRealsenseColorPublisher = ImageTransport(
            mNode,
            "/loomo/realsense/color/image_color",
            mRealsenseColorCamera,
            newStats("colour")
        )

        mFisheyeCamera = FisheyeCamera(mVision)
        mFisheyePublisher = ImageTransport(
            mNode,
            "/loomo/fisheye/image",
            mFisheyeCamera,
            newStats("fisheye")
        )

        // Start all the cameras, associating them with their streams
        startStream(mRealsenseDepthCamera, mRealsenseDepthPublisher)
        startStream(mRealsenseColorCamera, mRealsenseColorPublisher)
        startStream(mFisheyeCamera, mFisheyePublisher)
    }

    private fun newStats(name: String): StreamStats {
        val stats = StreamStats(name)
        mStatsReporter.register(stats)
        return stats
    }

    private fun startStream(camera: LoomoCamera, publisher: ImageTransport) {
        val stats = publisher.stats
        camera.startStream(object : Vision.FrameListener {
            override fun onNewFrame(streamType: Int, frame: Frame?) {
                val t0 = System.nanoTime()
                if (frame == null) {
                    Log.e(TAG, "Null-frame delivered from Loomo Vision Service!")
                    return
                }

                if (streamType != camera.getLoomoStreamType()) {
                    Log.e(
                        TAG,
                        "Unexpected stream type received from Loomo Vision Service: expected " + camera.getLoomoStreamType() + " got $streamType"
                    )
                    return
                }

                // Accounting first: a gap in the service's frame numbers means it captured frames
                // it never delivered, which happens when this callback is too slow (the service
                // has a 5-slot ring per stream) or when it dropped them for a non-increasing IMU
                // timestamp. Both are losses this app must know about.
                val frameNum = frame.info.frameNum
                val gap = stats.onSdkFrame(frameNum, frame.info.platformTimeStamp)
                if (gap > 0) {
                    Log.e(
                        StreamStats.TAG,
                        "${stats.name}: vision service skipped $gap frame(s) before frameNum=$frameNum " +
                            "(seen=${stats.sdkFramesSeen.get()} total_gap=${stats.sdkGapFrames.get()} " +
                            "stamp_us=${frame.info.platformTimeStamp})"
                    )
                }

                // Indicate we need the robot TF captured at this timestamp
                mTfPublisher.indicateTfNeededAtTime(mTfPublisher.captureTfContext(frame.info.platformTimeStamp))

                publisher.publish(frame)
                stats.recordCallbackNs(System.nanoTime() - t0)
            }
        })
    }

    private fun stopStream(camera: LoomoCamera) {
        camera.stopStream()
    }
}