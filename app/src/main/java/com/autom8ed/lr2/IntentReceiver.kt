package com.autom8ed.lr2

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.util.Log
import com.segway.robot.sdk.base.action.RobotAction

/**
 * Receives com.segway.robot.action.TO_ROBOT (sent when Loomo switches to robot mode or the
 * screen turns on) and brings the bridge's MainActivity back to the foreground.
 *
 * A BroadcastReceiver's Context is not an Activity, so startActivity() requires
 * FLAG_ACTIVITY_NEW_TASK. MainActivity uses the default (standard) launchMode, so
 * FLAG_ACTIVITY_REORDER_TO_FRONT is added to bring an already-running instance forward
 * instead of stacking a duplicate on top of it. Any failure is logged and swallowed so this
 * receiver can never take down the process.
 */
class IntentReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent?) {
        if (intent?.action != RobotAction.TransformEvent.ROBOT_MODE) {
            return
        }
        try {
            val activityIntent = Intent(context, MainActivity::class.java).apply {
                addFlags(
                    Intent.FLAG_ACTIVITY_NEW_TASK or
                        Intent.FLAG_ACTIVITY_REORDER_TO_FRONT
                )
            }
            context.startActivity(activityIntent)
        } catch (e: Exception) {
            Log.e(TAG, "Failed to bring MainActivity to front for ${intent.action}", e)
        }
    }

    companion object {
        private const val TAG = "IntentReceiver"
    }
}
