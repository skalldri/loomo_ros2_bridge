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
    // onSubscriptionStateChange() is only forwarded once a subclass that overrides it has
    // called enableSubscriptionStateCallbacks(); see that method.
    private var mCallbacksEnabled: Boolean = false

    open val TAG = "AdvancedPublisher - $mTopic"

    private val mMatchedEventHandler: EventHandler<*, *>? = mPublisher.createEventHandler(
        Matched.factory
    ) { status ->
        mSem.acquire()
        val oldHasSubscribers = mHasSubscribers
        mHasSubscribers = status!!.currentCount > 0

        if (mCallbacksEnabled && oldHasSubscribers != mHasSubscribers) {
            onSubscriptionStateChange(mHasSubscribers)
        }

        mSem.release()
    }

    /**
     * Subclasses that override onSubscriptionStateChange() must call this at the end of their
     * init block. The matched-event handler above is registered while this base class is being
     * constructed and runs on an executor thread, so with a subscriber already present (the
     * Jetson's DDS-Router is usually up before the app) it can fire before the subclass's own
     * fields exist: H264ImagePublisher crashed with an NPE on its not-yet-created Semaphore.
     * Until this is called the handler only records the subscriber state; calling it delivers
     * the current state once if a subscriber has matched in the meantime.
     */
    protected fun enableSubscriptionStateCallbacks() {
        mSem.acquire()
        mCallbacksEnabled = true
        if (mHasSubscribers) {
            onSubscriptionStateChange(true)
        }
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

    open fun onSubscriptionStateChange(hasSubscribers: Boolean) {

    }
}

