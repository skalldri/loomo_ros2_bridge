package com.autom8ed.lr2

import android.util.Log
import java.util.Deque
import java.util.LinkedList
import java.util.Queue
import kotlin.system.measureNanoTime

class PerfEvent(startTime: Long, endTime: Long, lastEndTime: Long) {
    public var mStartTime: Long = startTime
    public var mEndTime: Long = endTime
    public var mLastEndTime: Long = lastEndTime
}

class PerfCounter(name: String) {
    private val mName: String = name

    private var mStartTime: Long = 0
    private var mEndTime: Long = 0
    private var mResultsQueue: Deque<PerfEvent> = LinkedList<PerfEvent>();

    val mPrintIntervalSeconds: Float = 5.0f
    var mLastPrintTime: Long = 0

    init {

    }

    fun start() {
        mStartTime = System.nanoTime()

        if (mLastPrintTime == 0L) {
            mLastPrintTime = mStartTime
        }
    }

    fun stop() {
        val endTime: Long = System.nanoTime()

        // Calculate start-to-end time (this is the best rate if we ran again immediately)
        if (mStartTime != 0L && mEndTime != 0L && endTime > mStartTime && endTime > mEndTime) {

            mResultsQueue.push(PerfEvent(mStartTime, endTime, mEndTime))
        }

        mEndTime = endTime
        // Time to print again
        if (mEndTime > (mLastPrintTime + (mPrintIntervalSeconds * 1000000000))) {
            val numSamples = mResultsQueue.size

            var timeElapsedRunTimeAvg = 0.0f
            var timeElapsedRunRateAvg = 0.0f
            var timeElapsedExecutionRateAvg = 0.0f

            for (e in mResultsQueue) {
                val timeElapsedRunTime = (e.mEndTime - e.mStartTime).toFloat() / 1000000.0f
                val timeElapsedRunRate = 1.0f / (timeElapsedRunTime / 1000.0f)
                val timeElapsedExecutionRate = 1.0f / ((e.mEndTime - e.mLastEndTime).toFloat() / 1000000000.0f)

                timeElapsedRunTimeAvg += timeElapsedRunTime
                timeElapsedRunRateAvg += timeElapsedRunRate
                timeElapsedExecutionRateAvg += timeElapsedExecutionRate
            }

            timeElapsedRunTimeAvg /= numSamples
            timeElapsedRunRateAvg /= numSamples
            timeElapsedExecutionRateAvg /= numSamples

            Log.d(mName, "Run Time Rate (Avg, $mPrintIntervalSeconds s): $timeElapsedRunTimeAvg ms")
            Log.d(mName, "Best Possible Execution Rate (Avg, $mPrintIntervalSeconds s): $timeElapsedRunRateAvg")
            Log.d(mName, "Actual Execution Rate (Avg, $mPrintIntervalSeconds s): $timeElapsedExecutionRateAvg Hz")

            mLastPrintTime = mEndTime
        }
    }

    fun instant() {
        start()
        stop()
    }
}