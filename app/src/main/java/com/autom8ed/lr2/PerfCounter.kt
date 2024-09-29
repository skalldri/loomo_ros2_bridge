package com.autom8ed.lr2

import android.util.Log
import kotlin.system.measureNanoTime

class PerfCounter(name: String) {
    private val mName: String = name

    private var mStartTime: Long = 0
    private var mEndTime: Long = 0

    val mPrintIntervalSeconds: Float = 5.0f
    // var mLastPrintTime

    init {

    }

    fun start() {
        mStartTime = System.nanoTime()
    }

    fun stop() {
        val endTime: Long = System.nanoTime()

        // Calculate end-end time (this is our actual execution rate)
        if (mStartTime != 0L && endTime > mStartTime) {
            val timeElapsedRunTime = (endTime - mStartTime).toFloat() / 1000000.0f
            Log.d(mName, "Run Time Rate: $timeElapsedRunTime ms")
            val timeElapsedRunRate = 1.0f / (timeElapsedRunTime / 1000.0)
            Log.d(mName, "Best Possible Execution Rate: $timeElapsedRunRate")
        }

        // Calculate end-end time (this is our actual execution rate)
        if (mEndTime != 0L && endTime > mEndTime) {
            val timeElapsedExecutionRate = 1.0f / ((endTime - mEndTime).toFloat() / 1000000000.0f)
            Log.d(mName, "Actual Execution Rate: $timeElapsedExecutionRate Hz")
        }

        mEndTime = endTime
    }

    fun instant() {
        start()
        stop()
    }
}