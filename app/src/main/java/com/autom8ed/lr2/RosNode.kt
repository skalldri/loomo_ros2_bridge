package com.autom8ed.lr2

import android.util.Log
import org.ros2.rcljava.node.BaseComposableNode
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.timer.WallTimer
import java.util.concurrent.TimeUnit


class RosNode(name: String) : BaseComposableNode(name) {
    private var publisher: Publisher<std_msgs.msg.String>
    private val fisheyeTopic = "chatter"
    private val T = "RosNode"
    private var count = 0

    private var timer: WallTimer? = null
    init {
        publisher = node.createPublisher(std_msgs.msg.String::class.java, fisheyeTopic)
    }

    fun start() {
        Log.d(T, "TalkerNode::start()")
        if (this.timer != null) {
            this.timer!!.cancel()
        }
        this.count = 0
        this.timer = node.createWallTimer(500, TimeUnit.MILLISECONDS) {
            this.onTimer()
        }
    }

    private fun onTimer() {
        val msg = std_msgs.msg.String()
        msg.setData("Hello ROS2 from Android: " + this.count)
        this.count++
        publisher.publish(msg)
    }

    fun stop() {
        Log.d(T, "TalkerNode::stop()")
        if (this.timer != null) {
            this.timer!!.cancel()
        }
    }
}