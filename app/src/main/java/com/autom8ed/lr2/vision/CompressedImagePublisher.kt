package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.RosNode
import com.segway.robot.sdk.vision.frame.Frame
import org.ros2.rcljava.qos.QoSProfile
import java.io.ByteArrayOutputStream
import java.util.concurrent.TimeUnit

class CompressedImagePublisher(
    node: RosNode,
    topic: String,
    camera: LoomoCamera,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) : AdvancedPublisher<sensor_msgs.msg.CompressedImage>(
    sensor_msgs.msg.CompressedImage::class.java,
    node,
    topic,
    qos
) {
    private val mCamera: LoomoCamera = camera

    fun publish(bitmap: Bitmap, platformTimeStamp: Long) {
        if (hasSubscribers()) {
            val compressedOutStream: ByteArrayOutputStream = ByteArrayOutputStream()
            bitmap.compress(Bitmap.CompressFormat.PNG, 100, compressedOutStream)

            // Assign to ROS msg
            val msg = sensor_msgs.msg.CompressedImage()
            msg.data = compressedOutStream.toByteArray()

            msg.header.frameId = mCamera.getTfOpticalFrameId()
            msg.format = "png"

            msg.header.stamp.sec =
                TimeUnit.SECONDS.convert(platformTimeStamp, TimeUnit.MICROSECONDS)
                    .toInt()
            msg.header.stamp.nanosec =
                (platformTimeStamp % (1000 * 1000)).toInt() * (1000)

            publish(msg)
        }
    }
}