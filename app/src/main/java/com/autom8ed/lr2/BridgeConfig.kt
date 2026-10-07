package com.autom8ed.lr2

import android.content.Context
import android.content.Intent
import android.util.Log
import java.io.File
import java.io.FileInputStream
import java.util.Properties

/**
 * Runtime configuration of the bridge, read once at start-up.
 *
 * Sources, later ones overriding earlier ones:
 *  1. the compiled defaults below;
 *  2. `bridge.properties` in the app's external files directory
 *     (`/sdcard/Android/data/com.autom8ed.lr2/files/bridge.properties`, which `adb push` can
 *     write without any permission);
 *  3. the extras of the Intent that started the activity, for one-off experiments:
 *     `am start -n com.autom8ed.lr2/.MainActivity --es colour.raw true`.
 * The boot receiver starts the activity without extras, so a persistent change goes in the file.
 * The effective configuration is logged under the tag "BridgeConfig".
 *
 * Keys (all optional):
 *  - `depth.enabled`, `colour.enabled`, `fisheye.enabled`: run the stream at all (default true).
 *  - `depth.raw`, `colour.raw`, `fisheye.raw`: publish the raw sensor_msgs/Image (defaults
 *    true, false, true). Raw colour is 1.2 MB per frame, 37 MB/s at 30 fps, which the USB 2.0
 *    link cannot carry next to depth and fisheye; colour leaves the robot as H.264.
 *  - `colour.h264`: publish the H.264 stream (default true). Only the colour camera has one.
 *  - `queue.depth`: frames of jitter a worker may fall behind before frames are dropped and
 *    counted (default 8).
 *  - `stats.tagFrameNum`: carry `frameNum % 1000` in the low digits of header.stamp.nanosec
 *    so the receiver can check for gaps (default true); see FrameStamp.
 *  - `stats.periodS`: seconds between StreamStats report lines (default 5).
 *  - `dds.priority`: load the Fast DDS profile (assets/fastdds_profiles.xml) that sends TF,
 *    odometry and joint states ahead of the camera images (default true); see FastDdsProfile.
 */
class BridgeConfig private constructor(private val values: Map<String, String>, private val sources: String) {

    class Stream(val name: String, val enabled: Boolean, val raw: Boolean, val h264: Boolean) {
        override fun toString() = "$name(enabled=$enabled raw=$raw h264=$h264)"
    }

    val depth = stream("depth", raw = true, h264 = false)
    val colour = stream("colour", raw = false, h264 = true)
    val fisheye = stream("fisheye", raw = true, h264 = false)
    val queueDepth = int("queue.depth", 8, min = 1)
    val tagFrameNum = bool("stats.tagFrameNum", true)
    val statsPeriodS = int("stats.periodS", 5, min = 1)
    val ddsPriority = bool("dds.priority", true)

    private fun stream(name: String, raw: Boolean, h264: Boolean) = Stream(
        name,
        enabled = bool("$name.enabled", true),
        raw = bool("$name.raw", raw),
        h264 = bool("$name.h264", h264)
    )

    private fun bool(key: String, default: Boolean): Boolean {
        val v = values[key]?.trim()?.lowercase() ?: return default
        return when (v) {
            "true", "1", "yes", "on" -> true
            "false", "0", "no", "off" -> false
            else -> {
                Log.e(TAG, "$key=\"$v\" is not a boolean; using $default")
                default
            }
        }
    }

    private fun int(key: String, default: Int, min: Int): Int {
        val v = values[key]?.trim() ?: return default
        val n = v.toIntOrNull()
        if (n == null || n < min) {
            Log.e(TAG, "$key=\"$v\" is not an integer >= $min; using $default")
            return default
        }
        return n
    }

    fun log() {
        Log.i(TAG, "sources: $sources")
        Log.i(TAG, "streams: $depth $colour $fisheye")
        Log.i(TAG, "queue.depth=$queueDepth stats.tagFrameNum=$tagFrameNum stats.periodS=$statsPeriodS dds.priority=$ddsPriority")
        for (k in values.keys.sorted()) {
            if (k !in KNOWN_KEYS) Log.w(TAG, "unknown key \"$k\" ignored")
        }
    }

    companion object {
        const val TAG = "BridgeConfig"
        const val FILE_NAME = "bridge.properties"
        private val KNOWN_KEYS = setOf(
            "depth.enabled", "depth.raw", "depth.h264",
            "colour.enabled", "colour.raw", "colour.h264",
            "fisheye.enabled", "fisheye.raw", "fisheye.h264",
            "queue.depth", "stats.tagFrameNum", "stats.periodS", "dds.priority"
        )

        fun load(context: Context, intent: Intent?): BridgeConfig {
            val values = HashMap<String, String>()
            val sources = StringBuilder("defaults")

            val dir = context.getExternalFilesDir(null)
            val file = if (dir != null) File(dir, FILE_NAME) else null
            if (file != null && file.isFile) {
                try {
                    val p = Properties()
                    FileInputStream(file).use { p.load(it) }
                    for (name in p.stringPropertyNames()) values[name] = p.getProperty(name)
                    sources.append(", ").append(file.path).append(" (").append(p.size).append(" keys)")
                } catch (e: Exception) {
                    Log.e(TAG, "could not read ${file.path}; ignoring it", e)
                }
            } else {
                sources.append(", no ").append(file?.path ?: FILE_NAME)
            }

            val extras = intent?.extras
            if (extras != null && !extras.isEmpty) {
                var n = 0
                for (k in extras.keySet()) {
                    val v = extras.get(k) ?: continue
                    values[k] = v.toString()
                    n++
                }
                sources.append(", intent extras (").append(n).append(" keys)")
            }

            return BridgeConfig(values, sources.toString())
        }
    }
}
