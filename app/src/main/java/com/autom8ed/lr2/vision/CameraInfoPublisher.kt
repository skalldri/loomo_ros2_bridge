package com.autom8ed.lr2.vision

import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile

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

    fun publish(platformTimeStampUs: Long, frameNum: Int) {
        if (!hasSubscribers()) {
            stats?.topic(mTopic)?.skippedNoSubscriber?.incrementAndGet()
            return
        }
        val msg: sensor_msgs.msg.CameraInfo = mCamera.getCameraInfo()
        FrameStamp.apply(msg.header.stamp, platformTimeStampUs, frameNum)
        tryPublish(msg, frameNum)
    }
}