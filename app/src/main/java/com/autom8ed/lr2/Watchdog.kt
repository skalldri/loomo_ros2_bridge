package com.autom8ed.lr2

import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.qos.QoSProfile
import std_msgs.msg.Empty
import kotlin.concurrent.thread

class Watchdog(ctx: android.content.Context, node: RosNode, headInterface: HeadInterface) {

    val mNode: RosNode = node
    val mWatchdogPublisher: Publisher<std_msgs.msg.Empty> =
        mNode.node.createPublisher(std_msgs.msg.Empty::class.java, "/loomo/android_watchdog", QoSProfile.SENSOR_DATA)
    val mHeadInterface: HeadInterface = headInterface
    val mWatchdogPublisherThread: Thread
    val mWatchdogSubscriberThread: Thread

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
            while (true) {
                // Every second, check to ensure the Jetson is still connected
                Thread.sleep((1000.0f).toLong())

                // State has changed! Which direction did it change?
                if (lastJetsonAlive != mJetsonAlive) {
                    // Jetson has connected for the first time!
                    if (mJetsonAlive) {
                        mHeadInterface.setLightMode(mHeadInterface.HEAD_LIGHT_MODE_GREEN_FADE_IN_OUT)
                    }
                    // Jetson has disconnected, set error indication
                    else {
                        mHeadInterface.setLightMode(mHeadInterface.HEAD_LIGHT_MODE_RED_FLASHING)
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
        }
    }
}