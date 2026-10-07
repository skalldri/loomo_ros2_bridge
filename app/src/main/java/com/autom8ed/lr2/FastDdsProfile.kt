package com.autom8ed.lr2

import android.content.Context
import android.util.Log
import java.io.File

/**
 * Installs the Fast DDS XML profile shipped in assets/[ASSET] into the app's files directory,
 * where Fast DDS can read it through FASTRTPS_DEFAULT_PROFILES_FILE (it cannot read APK assets).
 * Copied on every start so an app update always takes effect.
 */
object FastDdsProfile {
    private const val TAG = "BridgeConfig"
    const val ASSET = "fastdds_profiles.xml"

    /** Returns the installed profile's path, or null (logged) if it could not be written. */
    fun install(context: Context): String? {
        val dest = File(context.filesDir, ASSET)
        return try {
            context.assets.open(ASSET).use { input -> dest.outputStream().use { input.copyTo(it) } }
            Log.i(TAG, "Fast DDS profile: ${dest.absolutePath}")
            dest.absolutePath
        } catch (e: Exception) {
            Log.e(TAG, "Fast DDS profile not installed, running with Fast DDS defaults", e)
            null
        }
    }
}
