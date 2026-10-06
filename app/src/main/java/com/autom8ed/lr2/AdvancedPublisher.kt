package com.autom8ed.lr2

import android.util.Log
import com.autom8ed.lr2.vision.StreamStats
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
    // Read lock-free on every frame; the semaphore only orders the matched-event handler and
    // enableSubscriptionStateCallbacks() against each other.
    @Volatile private var mHasSubscribers: Boolean = false

    /** Delivery accounting for the stream this publisher belongs to; set by the owning transport. */
    @Volatile var stats: StreamStats? = null
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
        tryPublish(msg, -1)
    }

    /**
     * Publishes if a subscriber is matched. Returns true only when rcl accepted the message.
     * Every other outcome is counted in [stats] (no subscriber, or a failure, which is also
     * logged with the frame number), so no frame disappears silently.
     */
    fun tryPublish(msg: MessageT, frameNum: Int): Boolean {
        val counters = stats?.topic(mTopic)
        if (!hasSubscribers()) {
            counters?.skippedNoSubscriber?.incrementAndGet()
            return false
        }
        return try {
            val t0 = System.nanoTime()
            mPublisher.publish(msg)
            val dtMs = (System.nanoTime() - t0) / 1e6
            if (dtMs > SLOW_PUBLISH_MS) {
                // A publish that takes longer than a few frame periods is a delivery problem in
                // the making (the frame queue behind it fills up); say which topic and how long.
                Log.w(TAG, "slow publish on $mTopic: ${"%.1f".format(dtMs)} ms (frameNum=$frameNum, #${counters?.published?.get() ?: -1})")
            }
            counters?.published?.incrementAndGet()
            true
        } catch (e: Exception) {
            val s = stats
            if (s != null) {
                s.recordPublishFailure(mTopic, frameNum, e)
            } else {
                Log.e(TAG, "publish failed on $mTopic (frameNum=$frameNum)", e)
            }
            false
        }
    }

    fun hasSubscribers(): Boolean = mHasSubscribers

    open fun onSubscriptionStateChange(hasSubscribers: Boolean) {

    }

    companion object {
        const val SLOW_PUBLISH_MS = 200.0
    }
}

