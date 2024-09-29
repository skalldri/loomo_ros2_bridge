package com.autom8ed.lr2

import android.util.Log
import java.text.DateFormat
import java.text.SimpleDateFormat
import java.util.Date
import java.util.concurrent.TimeUnit
import kotlin.math.abs


class TimeSync {
    var mInitialImuTimestampNs: Long = 0
    var mInitialSystemTimestampMs: Long = 0

    val TAG = "TimeSync";

    fun getRosTime(imuTimestampNs: Long, systemTimestampMs: Long): builtin_interfaces.msg.Time {
        if (mInitialImuTimestampNs == 0L && mInitialSystemTimestampMs == 0L) {
            mInitialImuTimestampNs = imuTimestampNs;
            mInitialSystemTimestampMs = systemTimestampMs;
        }

        val deltaSystemTimeNs = (systemTimestampMs - mInitialSystemTimestampMs) * (1000 * 1000);
        val deltaImuTimeNs = (imuTimestampNs - mInitialImuTimestampNs);
        // val driftNs = abs(deltaSystemTimeNs - deltaImuTimeNs);

        val driftRatio: Double = deltaSystemTimeNs.toDouble() / deltaImuTimeNs.toDouble();
        val driftCorrectedDeltaImuTimeNs = (deltaImuTimeNs * driftRatio).toLong();

        val currentSystemTimeNs: Long = (mInitialSystemTimestampMs * (1000 * 1000)) + driftCorrectedDeltaImuTimeNs;

        val time: builtin_interfaces.msg.Time = builtin_interfaces.msg.Time();
        time.sec = TimeUnit.SECONDS.convert(currentSystemTimeNs, TimeUnit.NANOSECONDS).toInt();
        time.nanosec = (currentSystemTimeNs % (1000 * 1000 * 1000)).toInt();

        return time;
    }
}