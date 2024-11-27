package com.autom8ed.lr2.vision

import android.util.Log
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.TfPublisher
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.vision.Vision
import com.segway.robot.sdk.vision.frame.Frame
import kotlinx.coroutines.delay
import java.util.concurrent.Semaphore

class CameraInterface(
    ctx: android.content.Context,
    node: RosNode,
    tfPublisher: TfPublisher
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

        // Setup the camera publishers
        mRealsenseDepthCamera = RealsenseDepthCamera(mVision)
        mRealsenseDepthPublisher = ImageTransport(
            mNode,
            "/loomo/realsense/depth/image_depth_rect",
            mRealsenseDepthCamera
        )

        mRealsenseColorCamera = RealsenseColorCamera(mVision)
        mRealsenseColorPublisher = ImageTransport(
            mNode,
            "/loomo/realsense/color/image_color",
            mRealsenseColorCamera
        )

        mFisheyeCamera = FisheyeCamera(mVision)
        mFisheyePublisher = ImageTransport(
            mNode,
            "/loomo/fisheye/image",
            mFisheyeCamera
        )

        // Start all the cameras, associating them with their streams
        startStream(mRealsenseDepthCamera, mRealsenseDepthPublisher)
        startStream(mRealsenseColorCamera, mRealsenseColorPublisher)
        startStream(mFisheyeCamera, mFisheyePublisher)
    }

    private fun startStream(camera: LoomoCamera, publisher: ImageTransport) {
        camera.startStream(object : Vision.FrameListener {
            override fun onNewFrame(streamType: Int, frame: Frame?) {
                if (frame == null) {
                    Log.e(TAG, "Null-frame delivered from Loomo Vision Service!")
                    return
                }

                if (streamType != camera.getLoomoStreamType()) {
                    Log.e(
                        TAG,
                        "Unexpected stream type received from Loomo Vision Service: expected " + mRealsenseDepthCamera.getLoomoStreamType() + " got $streamType"
                    )
                    return
                }

                // Indicate we need the robot TF captured at this timestamp
                mTfPublisher.indicateTfNeededAtTime(mTfPublisher.captureTfContext(frame.info.platformTimeStamp))

                publisher.publish(frame)
            }
        })
    }

    private fun stopStream(camera: LoomoCamera) {
        camera.stopStream()
    }
}