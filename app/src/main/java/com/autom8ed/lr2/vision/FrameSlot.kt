package com.autom8ed.lr2.vision

import com.segway.robot.sdk.vision.frame.Frame
import java.nio.ByteBuffer

/**
 * One camera frame copied out of the Vision SDK's shared-memory buffer, which is only valid
 * while the SDK callback runs. The buffer is a heap ByteBuffer sized exactly to the image so
 * `buffer.array()` can be handed to a sensor_msgs/Image without another copy. Slots live in a
 * [FrameQueue] pool and are reused, so steady state allocates nothing per frame.
 */
class FrameSlot(val capacityBytes: Int) {
    val buffer: ByteBuffer = ByteBuffer.allocate(capacityBytes)
    var frameNum: Int = -1
    var platformTimeStampUs: Long = 0
    var imuTimeStampUs: Long = 0
    var enqueuedAtNs: Long = 0

    /** Copies the SDK frame (one memcpy) and its metadata. */
    fun fill(frame: Frame) {
        val src = frame.byteBuffer.duplicate()
        src.rewind()
        if (src.remaining() > capacityBytes) {
            src.limit(capacityBytes)
        }
        buffer.clear()
        buffer.put(src)
        buffer.flip()
        frameNum = frame.info.frameNum
        platformTimeStampUs = frame.info.platformTimeStamp
        imuTimeStampUs = frame.info.getIMUTimeStamp()
        enqueuedAtNs = System.nanoTime()
    }

    /** A read-only view positioned at the start, for consumers that advance a position. */
    fun view(): ByteBuffer = buffer.duplicate().also { it.rewind() }
}
