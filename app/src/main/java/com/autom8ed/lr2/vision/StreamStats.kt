package com.autom8ed.lr2.vision

import android.util.Log
import java.util.concurrent.ConcurrentHashMap
import java.util.concurrent.atomic.AtomicInteger
import java.util.concurrent.atomic.AtomicLong

/**
 * Delivery accounting for one camera stream, from the Vision SDK callback to the ROS publishers.
 *
 * Cumulative counters are never reset, so two readings taken at any two times can be diffed
 * (on the robot from logcat, or against what the Jetson received in the same interval).
 * Window maxima are reset by StatsReporter after each report. Everything is lock-free because
 * the SDK callback (binder thread), the frame worker and the H.264 drain thread all write here.
 */
class StreamStats(val name: String) {

    /** Per ROS topic: successful publishes, publishes skipped for lack of a subscriber, failures. */
    class TopicCounters(val topic: String) {
        val published = AtomicLong()
        val skippedNoSubscriber = AtomicLong()
        val failures = AtomicLong()
    }

    /** Immutable copy for reporting. */
    class Snapshot(
        val sdkFramesSeen: Long,
        val sdkGapFrames: Long,
        val sdkFrameNumRegressions: Long,
        val queued: Long,
        val queueDropped: Long,
        val rateSkipped: Long,
        val publishFailures: Long,
        val encoderRestarts: Long,
        val callbackNsMax: Long,
        val workerNsMax: Long,
        val queueDepthMax: Int,
        val lastFrameNum: Int,
        val lastPlatformStampUs: Long,
        val topics: Map<String, Triple<Long, Long, Long>>  // published, skippedNoSubscriber, failures
    )

    // Cumulative.
    val sdkFramesSeen = AtomicLong()
    /** Frames the vision service numbered but never delivered to us (positive frameNum jumps). */
    val sdkGapFrames = AtomicLong()
    /** frameNum went backwards (service restart); the gap baseline resets. */
    val sdkFrameNumRegressions = AtomicLong()
    val queued = AtomicLong()
    val queueDropped = AtomicLong()
    val rateSkipped = AtomicLong()
    val publishFailures = AtomicLong()
    val encoderRestarts = AtomicLong()

    // Window maxima (reset each report).
    private val callbackNsMax = AtomicLong()
    private val workerNsMax = AtomicLong()
    private val queueDepthMax = AtomicInteger()

    @Volatile var lastFrameNum: Int = -1
        private set
    @Volatile var lastPlatformStampUs: Long = 0
        private set

    private val topics = ConcurrentHashMap<String, TopicCounters>()

    fun topic(topic: String): TopicCounters = topics.getOrPut(topic) { TopicCounters(topic) }

    /** Called first thing in the SDK callback. Returns the number of frames skipped before this one. */
    fun onSdkFrame(frameNum: Int, platformTimeStampUs: Long): Int {
        sdkFramesSeen.incrementAndGet()
        val last = lastFrameNum
        var gap = 0
        if (last >= 0) {
            if (frameNum > last + 1) {
                gap = frameNum - last - 1
                sdkGapFrames.addAndGet(gap.toLong())
            } else if (frameNum <= last) {
                sdkFrameNumRegressions.incrementAndGet()
            }
        }
        lastFrameNum = frameNum
        lastPlatformStampUs = platformTimeStampUs
        return gap
    }

    fun recordCallbackNs(ns: Long) = maxInto(callbackNsMax, ns)
    fun recordWorkerNs(ns: Long) = maxInto(workerNsMax, ns)
    fun recordQueueDepth(depth: Int) {
        while (true) {
            val cur = queueDepthMax.get()
            if (depth <= cur || queueDepthMax.compareAndSet(cur, depth)) return
        }
    }

    fun recordPublishFailure(topic: String, frameNum: Int, e: Throwable) {
        publishFailures.incrementAndGet()
        topic(topic).failures.incrementAndGet()
        // Loud by design: a lost frame must never be silent. StatsReporter summarises the totals.
        Log.e(TAG, "publish failed: stream=$name topic=$topic frameNum=$frameNum " +
            "seen=${sdkFramesSeen.get()} failures=${publishFailures.get()}", e)
    }

    fun snapshot(): Snapshot = Snapshot(
        sdkFramesSeen.get(), sdkGapFrames.get(), sdkFrameNumRegressions.get(),
        queued.get(), queueDropped.get(), rateSkipped.get(), publishFailures.get(),
        encoderRestarts.get(), callbackNsMax.get(), workerNsMax.get(), queueDepthMax.get(),
        lastFrameNum, lastPlatformStampUs,
        topics.mapValues { Triple(it.value.published.get(), it.value.skippedNoSubscriber.get(), it.value.failures.get()) }
    )

    fun resetWindowMaxima() {
        callbackNsMax.set(0)
        workerNsMax.set(0)
        queueDepthMax.set(0)
    }

    private fun maxInto(target: AtomicLong, value: Long) {
        while (true) {
            val cur = target.get()
            if (value <= cur || target.compareAndSet(cur, value)) return
        }
    }

    companion object {
        const val TAG = "StreamStats"
    }
}
