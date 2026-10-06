package com.autom8ed.lr2.vision

import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile

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

    // The rcljava publish: JNI copy, CDR serialisation and, in synchronous mode, the UDP sends.
    // 5 s windows in logcat.
    private val mPublishPerf: PerfCounter = PerfCounter("RawImage - $mTopic - publish")

    fun publish(slot: FrameSlot, platformTimeStamp: Long, frameNum: Int) {
        if (!hasSubscribers()) {
            stats?.topic(mTopic)?.skippedNoSubscriber?.incrementAndGet()
            return
        }

        val msg = sensor_msgs.msg.Image()
        // The slot's backing array is exactly the image: no copy on the Java side
        msg.data = slot.buffer.array()

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