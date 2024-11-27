package com.autom8ed.lr2

import android.util.Log
import org.ros2.rcljava.consumers.Consumer
import org.ros2.rcljava.events.EventHandler
import org.ros2.rcljava.interfaces.MessageDefinition
import org.ros2.rcljava.publisher.Publisher
import org.ros2.rcljava.publisher.statuses.Matched
import org.ros2.rcljava.publisher.statuses.LivelinessLost
import org.ros2.rcljava.qos.QoSProfile
import java.util.concurrent.Semaphore

open class AdvancedPublisher<MessageT : MessageDefinition?>(
    type: Class<MessageT>, node: RosNode, topic: String, qos: QoSProfile = QoSProfile.DEFAULT
) {
    private val mNode: RosNode = node
    protected val mTopic: String = topic
    private val mQos: QoSProfile = qos
    private val mPublisher: Publisher<MessageT> = mNode.node.createPublisher(type, mTopic, mQos)
    private val mSem: Semaphore = Semaphore(1)
    private var mHasSubscribers: Boolean = false

    open val TAG = "AdvancedPublisher - $mTopic"

    private val mMatchedEventHandler: EventHandler<*, *>? = mPublisher.createEventHandler(
        Matched.factory
    ) { status ->
        mSem.acquire()
        mHasSubscribers = status!!.currentCount > 0
        mSem.release()
    }

    open fun publish(msg: MessageT) {
        if (hasSubscribers()) {
            mPublisher.publish(msg)
        }
    }

    fun hasSubscribers(): Boolean {
        mSem.acquire()
        val hasSubs = mHasSubscribers
        mSem.release()
        return hasSubs
    }
}

