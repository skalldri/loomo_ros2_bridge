package com.autom8ed.lr2.vision

import android.util.Log

/**
 * Resolves the native entry points of every message class the camera streams publish, before
 * the first frame.
 *
 * The first publish on each topic used to take 0.25-2.2 s, serialised across the three frame
 * workers, and the frame queues overflowed (~50-90 drops per stream at every start). Loading the
 * typesupport library (what constructing a message does) was not enough: rcljava's publish path
 * calls the class's static natives (`getDestructor()` first), and ART links each native method
 * lazily on its first call by probing every loaded library with dlsym. This app has ~1000
 * native libraries, so that probe is slow and runs under the loader lock. Calling the natives
 * once here moves that cost to start-up, and the measured time is logged so a regression shows.
 */
object MessageWarmup {
    private const val TAG = "MessageWarmup"
    @Volatile private var done = false

    @Synchronized
    fun warm() {
        if (done) return
        done = true
        val t0 = System.nanoTime()
        time("Image") {
            sensor_msgs.msg.Image()
            sensor_msgs.msg.Image.getDestructor()
            sensor_msgs.msg.Image.getFromJavaConverter()
            sensor_msgs.msg.Image.getToJavaConverter()
            sensor_msgs.msg.Image.getTypeSupport()
        }
        time("CameraInfo") {
            sensor_msgs.msg.CameraInfo()
            sensor_msgs.msg.CameraInfo.getDestructor()
            sensor_msgs.msg.CameraInfo.getFromJavaConverter()
            sensor_msgs.msg.CameraInfo.getToJavaConverter()
            sensor_msgs.msg.CameraInfo.getTypeSupport()
        }
        time("CompressedImage") {
            sensor_msgs.msg.CompressedImage()
            sensor_msgs.msg.CompressedImage.getDestructor()
            sensor_msgs.msg.CompressedImage.getFromJavaConverter()
            sensor_msgs.msg.CompressedImage.getToJavaConverter()
            sensor_msgs.msg.CompressedImage.getTypeSupport()
        }
        time("Header") {
            std_msgs.msg.Header()
            std_msgs.msg.Header.getDestructor()
            std_msgs.msg.Header.getFromJavaConverter()
            std_msgs.msg.Header.getToJavaConverter()
            std_msgs.msg.Header.getTypeSupport()
        }
        time("RegionOfInterest") {
            sensor_msgs.msg.RegionOfInterest()
            sensor_msgs.msg.RegionOfInterest.getDestructor()
            sensor_msgs.msg.RegionOfInterest.getFromJavaConverter()
            sensor_msgs.msg.RegionOfInterest.getToJavaConverter()
            sensor_msgs.msg.RegionOfInterest.getTypeSupport()
        }
        time("Time") {
            builtin_interfaces.msg.Time()
            builtin_interfaces.msg.Time.getDestructor()
            builtin_interfaces.msg.Time.getFromJavaConverter()
            builtin_interfaces.msg.Time.getToJavaConverter()
            builtin_interfaces.msg.Time.getTypeSupport()
        }
        Log.i(TAG, "message natives resolved in ${"%.1f".format((System.nanoTime() - t0) / 1e6)} ms")
    }

    private inline fun time(name: String, block: () -> Unit) {
        val t0 = System.nanoTime()
        block()
        Log.i(TAG, "$name: ${"%.1f".format((System.nanoTime() - t0) / 1e6)} ms")
    }
}
