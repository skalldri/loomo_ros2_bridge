package com.autom8ed.lr2.vision

import android.util.Log
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.TimeSync
import com.segway.robot.sdk.vision.frame.Frame
import org.ros2.rcljava.qos.QoSProfile
import java.util.concurrent.TimeUnit

class CameraInfoPublisher(
    node: RosNode,
    topic: String,
    camera: LoomoCamera,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) : AdvancedPublisher<sensor_msgs.msg.CameraInfo>(
    sensor_msgs.msg.CameraInfo::class.java,
    node,
    topic,
    qos
) {

    private val mCamera: LoomoCamera = camera

    fun publish(frame: Frame) {
        if (hasSubscribers()) {
            val msg: sensor_msgs.msg.CameraInfo = mCamera.getCameraInfo()

            msg.header.stamp.sec = TimeUnit.SECONDS.convert(frame.info.platformTimeStamp, TimeUnit.MICROSECONDS).toInt();
            msg.header.stamp.nanosec = (frame.info.platformTimeStamp % (1000 * 1000)).toInt() * (1000);

            publish(msg)
        }
    }
}