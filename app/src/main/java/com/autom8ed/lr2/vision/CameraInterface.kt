package com.autom8ed.lr2.vision

import android.util.Log
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.BridgeConfig
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
    statsReporter: StatsReporter,
    config: BridgeConfig
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
    private val mConfig: BridgeConfig = config
    private val mWorkers = ArrayList<FrameWorker>()
    private val mCameras = ArrayList<LoomoCamera>()

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

        // One camera, transport and delivery accounting per enabled stream. Only the depth
        // stream triggers a TF/odometry capture: all three used to, which published the same
        // robot pose three times per frame period (~90 Hz of /tf) from inside the camera callbacks.
        //
        // Everything is constructed before any stream is started, and the streams are then
        // started back to back. The vision service takes ~1 s to bring the RealSense up after the
        // first startListenFrame(); a request for another stream that arrives during that window
        // is answered "nothing changed, will not restart" and that stream never delivers a frame
        // (seen on 2026-10-06 when the starts were interleaved with the ~2 s of set-up work).
        val streams = ArrayList<Triple<LoomoCamera, ImageTransport, Boolean>>()
        for ((cfg, topic, triggersTf) in listOf(
            Triple(mConfig.depth, "/loomo/realsense/depth/image_depth_rect", true),
            Triple(mConfig.colour, "/loomo/realsense/color/image_color", false),
            Triple(mConfig.fisheye, "/loomo/fisheye/image", false)
        )) {
            if (!cfg.enabled) {
                Log.w(TAG, "${cfg.name} stream disabled by configuration")
                continue
            }
            val camera: LoomoCamera = when (cfg.name) {
                "depth" -> RealsenseDepthCamera(mVision)
                "colour" -> RealsenseColorCamera(mVision)
                else -> FisheyeCamera(mVision)
            }
            streams.add(Triple(camera, ImageTransport(mNode, topic, camera, newStats(cfg.name), cfg), triggersTf))
        }
        for ((camera, transport, triggersTf) in streams) {
            startStream(camera, transport, triggersTf)
        }
    }

    /** Stops the streams and workers and releases the vision service (never done before: the
     *  service exhausted its buffers after many app restarts without an unbind). */
    fun stop() {
        for (c in mCameras) {
            try { c.stopStream() } catch (e: Exception) { Log.w(TAG, "stopStream failed", e) }
        }
        mCameras.clear()
        for (w in mWorkers) w.stop()
        mWorkers.clear()
        try { mVision.unbindService() } catch (e: Exception) { Log.w(TAG, "unbindService failed", e) }
    }

    private fun newStats(name: String): StreamStats {
        val stats = StreamStats(name)
        mStatsReporter.register(stats)
        return stats
    }

    private fun startStream(camera: LoomoCamera, publisher: ImageTransport, triggersTf: Boolean) {
        val stats = publisher.stats
        val res = camera.getResolution()
        val queue = FrameQueue(stats, mConfig.queueDepth, res.mWidth * res.mHeight * res.mPixelBytes)
        val worker = FrameWorker(stats.name, queue, publisher, stats)
        mWorkers.add(worker)
        worker.start()
        mCameras.add(camera)

        // The listener runs inside the vision service's binder call: the service waits for it to
        // return and overwrites its 5-slot ring if we are slow. So this does nothing but account,
        // copy the frame into a pooled slot and hand it to the worker thread.
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
                // it never delivered, which happens when this callback is too slow or when the
                // service dropped them for a non-increasing IMU timestamp. Both are losses this
                // app must know about.
                val frameNum = frame.info.frameNum
                val gap = stats.onSdkFrame(frameNum, frame.info.platformTimeStamp)
                if (gap > 0) {
                    Log.e(
                        StreamStats.TAG,
                        "${stats.name}: vision service skipped $gap frame(s) before frameNum=$frameNum " +
                            "(seen=${stats.sdkFramesSeen.get()} total_gap=${stats.sdkGapFrames.get()} " +
                            "stamp_us=${frame.info.platformTimeStamp} cb_last_ms=${(System.nanoTime() - t0) / 1e6})"
                    )
                }

                queue.offer(frame)

                if (triggersTf) {
                    mTfPublisher.indicateTfNeededAtTime(frame.info.platformTimeStamp)
                }
                stats.recordCallbackNs(System.nanoTime() - t0)
            }
        })
    }

    private fun stopStream(camera: LoomoCamera) {
        camera.stopStream()
    }
}