package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.TfPublisherConstants
import com.autom8ed.lr2.TimeSync
import com.segway.robot.sdk.vision.frame.Frame
import org.ros2.rcljava.qos.QoSProfile
import java.nio.ByteBuffer
import java.util.concurrent.TimeUnit

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

    fun publish(bitmap: Bitmap, platformTimeStamp: Long) {
        if (hasSubscribers()) {
            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer.clear()

            // Copy from Bitmap -> Buffer
            bitmap.copyPixelsToBuffer(mByteBuffer)

            val msg = sensor_msgs.msg.Image()
            // Assign to ROS msg
            msg.data = mByteBuffer.array()

            msg.header.frameId = mCamera.getTfOpticalFrameId()
            msg.encoding = mCamera.getImageType().getRosEncoding()
            msg.height = mCamera.getResolution().mHeight
            msg.width = mCamera.getResolution().mWidth
            msg.step = mCamera.getResolution().mWidth * mCamera.getResolution().mPixelBytes

            // msg.header.stamp = timeSync.getRosTime(frame.timestamp, frame.tid)
            msg.header.stamp.sec =
                TimeUnit.SECONDS.convert(platformTimeStamp, TimeUnit.MICROSECONDS)
                    .toInt()
            msg.header.stamp.nanosec =
                (platformTimeStamp % (1000 * 1000)).toInt() * (1000)

            publish(msg)
        }
    }
}