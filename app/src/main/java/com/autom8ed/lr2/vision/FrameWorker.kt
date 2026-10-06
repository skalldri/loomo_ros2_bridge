package com.autom8ed.lr2.vision

import android.util.Log

/**
 * One thread per camera stream: takes frames from the [FrameQueue], runs the stream's
 * [ImageTransport] (camera info, raw image, H.264 feed) and returns the slot. All the work that
 * used to run inside the SDK callback runs here, so the callback is a copy and a queue offer.
 */
class FrameWorker(
    private val name: String,
    private val queue: FrameQueue,
    private val transport: ImageTransport,
    private val stats: StreamStats
) {
    @Volatile private var running = false
    private var thread: Thread? = null

    fun start() {
        if (running) return
        running = true
        thread = Thread({ loop() }, "frame-$name").apply { start() }
    }

    fun stop() {
        running = false
        thread?.interrupt()
        thread?.join(2000)
        thread = null
    }

    private fun loop() {
        while (running) {
            val slot = try {
                queue.workerStage = "waiting"
                queue.take()
            } catch (e: InterruptedException) {
                return
            }
            val t0 = System.nanoTime()
            try {
                queue.workerStage = "publishing frameNum=${slot.frameNum}"
                transport.publish(slot)
            } catch (t: Throwable) {
                // tryPublish() already counts rcl failures; this catches everything else so the
                // worker never dies silently.
                stats.publishFailures.incrementAndGet()
                Log.e(StreamStats.TAG, "$name: worker failed on frameNum=${slot.frameNum}", t)
            } finally {
                queue.release(slot)
            }
            val dt = System.nanoTime() - t0
            queue.lastWorkerNs = dt
            stats.recordWorkerNs(dt)
        }
    }
}
