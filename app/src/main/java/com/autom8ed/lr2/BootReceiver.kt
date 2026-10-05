package com.autom8ed.lr2

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.util.Log

/**
 * Starts the bridge when the Loomo finishes booting, so the robot is usable without anyone
 * tapping the app icon (the Jetson side comes up on its own via systemd; the Loomo side did not).
 * Needs RECEIVE_BOOT_COMPLETED (declared in the manifest) and, on Android 5.1, the app must have
 * been launched at least once since install so it is out of the stopped state.
 */
class BootReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent?) {
        if (intent?.action != Intent.ACTION_BOOT_COMPLETED) {
            return
        }
        try {
            val activityIntent = Intent(context, MainActivity::class.java).apply {
                addFlags(Intent.FLAG_ACTIVITY_NEW_TASK)
            }
            context.startActivity(activityIntent)
            Log.i(TAG, "Boot completed; started MainActivity")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start MainActivity on boot", e)
        }
    }

    companion object {
        private const val TAG = "BootReceiver"
    }
}
