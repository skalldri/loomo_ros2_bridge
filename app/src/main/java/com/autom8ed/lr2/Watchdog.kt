package com.autom8ed.lr2

import android.util.Log
import org.ros2.rcljava.publisher.Publisher
import std_msgs.msg.Empty
import kotlin.concurrent.thread

/**
 * Heartbeats with the Jetson: sends /loomo/android_watchdog every 0.5 s, and shows on the head
 * light whether /loomo/jetson_watchdog (sent every 0.5 s) still arrives. Every received heartbeat
 * also goes to [onHeartbeat], which feeds DriveGate: driving stops 1.5 s after the last one.
 */
class Watchdog(
    ctx: android.content.Context,
    node: RosNode,
    headInterface: HeadInterface,
    private val onHeartbeat: () -> Unit = {},
) {

    val mNode: RosNode = node
    // RELIABLE like its consumer, the Jetson's loomo_watchdog (rclpy default QoS). With the
    // previous best-effort writer the heartbeat never crossed DDS-Router to domain 0 (2026-10-12:
    // 2 Hz on domain 1, 0 on domain 0), so the Jetson could not tell whether the app was alive.
    val mWatchdogPublisher: Publisher<std_msgs.msg.Empty> =
        mNode.node.createPublisher(
            std_msgs.msg.Empty::class.java,
            "/loomo/android_watchdog"
        )
    val mHeadInterface: HeadInterface = headInterface
    val mWatchdogPublisherThread: Thread
    val mWatchdogSubscriberThread: Thread

    // Written by the executor thread, read and reset by the check thread.
    @Volatile
    var mJetsonAlive: Boolean = false

    init {
        mWatchdogPublisherThread = thread() {
            while (true) {
                // Every 500ms, publish an Empty message indicating we are still alive and connected
                // this is 2x the rate of the Jetson watchdog, which (per Nyquist theorem) is fast enough
                // to ensure the Jetson never looses track of us, assuming no packet loss
                mWatchdogPublisher.publish(std_msgs.msg.Empty())
                Thread.sleep((500.0f).toLong())
            }
        }

        mWatchdogSubscriberThread = thread() {
            var lastJetsonAlive: Boolean = false
            // Seconds without a Jetson heartbeat; logged every MISSING_LOG_S so a silent link shows
            // in logcat (the LEDs stay white if the heartbeat never arrived, red if it stopped).
            var missingS = 0
            while (true) {
                // Every second, check to ensure the Jetson is still connected
                Thread.sleep((1000.0f).toLong())

                // State has changed! Which direction did it change?
                if (lastJetsonAlive != mJetsonAlive) {
                    // Jetson has connected for the first time!
                    if (mJetsonAlive) {
                        Log.i(TAG, "Jetson heartbeat received after ${missingS} s without")
                        mHeadInterface.setLightMode(mHeadInterface.HEAD_LIGHT_MODE_GREEN_FADE_IN_OUT)
                    }
                    // Jetson has disconnected, set error indication
                    else {
                        Log.e(TAG, "Jetson heartbeat lost")
                        mHeadInterface.setLightMode(mHeadInterface.HEAD_LIGHT_MODE_RED_FLASHING)
                    }
                }
                if (mJetsonAlive) {
                    missingS = 0
                } else {
                    missingS++
                    if (missingS % MISSING_LOG_S == 0) {
                        Log.e(TAG, "no Jetson heartbeat on /loomo/jetson_watchdog for ${missingS} s")
                    }
                }

                // Capture the last Jetson alive value
                lastJetsonAlive = mJetsonAlive

                // Un-alive the Jetson to reset the watchdog
                mJetsonAlive = false
            }
        }

        // Every time we get a message from the Jetson, indicate that it's alive again
        mNode.node.createSubscription(std_msgs.msg.Empty::class.java, "/loomo/jetson_watchdog") {
            mJetsonAlive = true
            onHeartbeat()
        }
    }

    companion object {
        const val TAG = "Watchdog"
        private const val MISSING_LOG_S = 30
    }
}
