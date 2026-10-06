package com.autom8ed.lr2.vision

import android.util.Log
import com.segway.robot.sdk.vision.frame.Frame
import java.util.concurrent.ArrayBlockingQueue

/**
 * Bounded hand-off between the Vision SDK callback (binder thread) and a stream's
 * [FrameWorker], backed by a fixed pool of [FrameSlot]s.
 *
 * Policy on overflow: the incoming frame is dropped, counted in [StreamStats.queueDropped] and
 * logged at error level with everything useful for diagnosis. The callback never blocks: the
 * vision service holds its 5-slot ring open until we return and starts overwriting frames if we
 * are slow, which would only move the loss out of sight.
 */
class FrameQueue(private val stats: StreamStats, val depth: Int, slotBytes: Int) {
    private val free = ArrayBlockingQueue<FrameSlot>(depth + 1)
    private val ready = ArrayBlockingQueue<FrameSlot>(depth + 1)
    @Volatile var lastWorkerNs: Long = 0
    @Volatile var workerStage: String = "idle"
    private var lastDropLogNs: Long = 0
    private var suppressedDropLogs: Int = 0

    init {
        repeat(depth + 1) { free.add(FrameSlot(slotBytes)) }
    }

    /** Called from the SDK callback. Returns false if the frame had to be dropped. */
    fun offer(frame: Frame): Boolean {
        val slot = free.poll()
        if (slot == null) {
            dropped(frame)
            return false
        }
        slot.fill(frame)
        stats.queued.incrementAndGet()
        ready.add(slot)   // cannot fail: free + ready together hold exactly depth + 1 slots
        stats.recordQueueDepth(ready.size)
        return true
    }

    /** Worker side: blocks until a frame is ready. */
    fun take(): FrameSlot = ready.take()

    /** Worker side: returns a slot to the pool once it is fully published. */
    fun release(slot: FrameSlot) {
        free.add(slot)
    }

    val size: Int get() = ready.size

    private fun dropped(frame: Frame) {
        val total = stats.queueDropped.incrementAndGet()
        val now = System.nanoTime()
        // The counter is exact; the detailed line is rate-limited to one per second.
        if (now - lastDropLogNs < 1_000_000_000L) {
            suppressedDropLogs++
            return
        }
        Log.e(
            StreamStats.TAG,
            "${stats.name}: DROPPED frameNum=${frame.info.frameNum} stamp_us=${frame.info.platformTimeStamp}: " +
                "queue full (${ready.size}/$depth) while worker in '$workerStage' " +
                "(last worker time ${lastWorkerNs / 1e6} ms); drops total=$total, " +
                "$suppressedDropLogs more since last line; seen=${stats.sdkFramesSeen.get()} " +
                "queued=${stats.queued.get()} svc_gap=${stats.sdkGapFrames.get()} fail=${stats.publishFailures.get()}"
        )
        lastDropLogNs = now
        suppressedDropLogs = 0
    }
}
