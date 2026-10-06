package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile
import java.nio.ByteBuffer

class BaseImagePublisher(
    node: RosNode,
    topic: String,
    camera: LoomoCamera,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) : AdvancedPublisher<sensor_msgs.msg.Image>(
    sensor_msgs.msg.Image::class.java,
    node,
    topic,
    qos
) {

    private val mCamera: LoomoCamera = camera

    private val mByteBuffer: ByteBuffer =
        ByteBuffer.allocate(mCamera.getResolution().mWidth * mCamera.getResolution().mHeight * mCamera.getResolution().mPixelBytes)

    // Where the time goes: the Bitmap -> ByteBuffer copy vs the rcljava publish (JNI copy, CDR
    // serialisation and, in synchronous mode, the UDP sends). 5 s windows in logcat.
    private val mCopyPerf: PerfCounter = PerfCounter("RawImage - $mTopic - copyPixelsToBuffer")
    private val mPublishPerf: PerfCounter = PerfCounter("RawImage - $mTopic - publish")

    fun publish(bitmap: Bitmap, platformTimeStamp: Long, frameNum: Int) {
        if (!hasSubscribers()) {
            stats?.topic(mTopic)?.skippedNoSubscriber?.incrementAndGet()
            return
        }

        mCopyPerf.start()
        // Reset position within the byte-buffer so that we overwrite everything
        mByteBuffer.clear()
        // Copy from Bitmap -> Buffer
        bitmap.copyPixelsToBuffer(mByteBuffer)
        mCopyPerf.stop()

        val msg = sensor_msgs.msg.Image()
        // Assign to ROS msg
        msg.data = mByteBuffer.array()

        msg.header.frameId = mCamera.getTfOpticalFrameId()
        msg.encoding = mCamera.getImageType().getRosEncoding()
        msg.height = mCamera.getResolution().mHeight
        msg.width = mCamera.getResolution().mWidth
        msg.step = mCamera.getResolution().mWidth * mCamera.getResolution().mPixelBytes
        FrameStamp.apply(msg.header.stamp, platformTimeStamp, frameNum)

        mPublishPerf.start()
        tryPublish(msg, frameNum)
        mPublishPerf.stop()
    }
}