package com.autom8ed.lr2.vision

import java.util.concurrent.TimeUnit

/**
 * Converts a frame's timestamp (microseconds) into a ROS header stamp. ImageTransport has already
 * moved the platform timestamp to the Jetson's clock (TimeSync.toJetsonUs), once per frame, so the
 * camera_info, raw image and H.264 messages of a frame carry the same stamp even if the clock
 * offset estimate changes between them.
 *
 * The microsecond source leaves the low three digits of `nanosec` at zero. With
 * [TAG_FRAME_NUM] on, those digits carry `frameNum % 1000`: every Image, CameraInfo and
 * CompressedImage of a frame then shares an exact per-frame identity that the Jetson-side
 * probe can check for gaps without any extra topic, at the cost of perturbing the stamp by
 * less than a microsecond. Documented in the README; turn it off for timing-sensitive consumers.
 */
object FrameStamp {
    @Volatile var TAG_FRAME_NUM: Boolean = true

    fun apply(stamp: builtin_interfaces.msg.Time, stampUs: Long, frameNum: Int) {
        stamp.sec = TimeUnit.SECONDS.convert(stampUs, TimeUnit.MICROSECONDS).toInt()
        var nanosec = (stampUs % 1_000_000L).toInt() * 1000
        if (TAG_FRAME_NUM && frameNum >= 0) {
            nanosec += Math.floorMod(frameNum, 1000)
        }
        stamp.nanosec = nanosec
    }
}
