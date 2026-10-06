package com.autom8ed.lr2.vision

import java.util.concurrent.TimeUnit

/**
 * Converts the Loomo's platform timestamp (microseconds) into a ROS header stamp.
 *
 * The microsecond source leaves the low three digits of `nanosec` at zero. With
 * [TAG_FRAME_NUM] on, those digits carry `frameNum % 1000`: every Image, CameraInfo and
 * CompressedImage of a frame then shares an exact per-frame identity that the Jetson-side
 * probe can check for gaps without any extra topic, at the cost of perturbing the stamp by
 * less than a microsecond. Documented in the README; turn it off for timing-sensitive consumers.
 */
object FrameStamp {
    @Volatile var TAG_FRAME_NUM: Boolean = true

    fun apply(stamp: builtin_interfaces.msg.Time, platformTimeStampUs: Long, frameNum: Int) {
        stamp.sec = TimeUnit.SECONDS.convert(platformTimeStampUs, TimeUnit.MICROSECONDS).toInt()
        var nanosec = (platformTimeStampUs % 1_000_000L).toInt() * 1000
        if (TAG_FRAME_NUM && frameNum >= 0) {
            nanosec += Math.floorMod(frameNum, 1000)
        }
        stamp.nanosec = nanosec
    }
}
