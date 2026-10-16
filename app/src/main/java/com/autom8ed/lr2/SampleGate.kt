package com.autom8ed.lr2

/**
 * Decides whether an SDK sample is new, by its raw timestamp (Loomo wall clock, microseconds,
 * before [TimeSync] conversion).
 *
 * The TF/odometry capture runs once per depth frame (30 Hz), but `Sensor.querySensorData` only
 * returns the latest snapshot of each source, and the Segway services update POSE_2D, BASE_IMU and
 * HEAD_WORLD_IMU at 20 Hz each (50.0 ms apart, whole-millisecond stamps, each at its own phase;
 * ~2% of Pose2D intervals are 100 ms). Publishing every capture repeated ~33% of the samples: the
 * same stamp twice on /loomo/odom and /tf, and after the clock sync (#23) re-converted copies that
 * differ by a fraction of a millisecond, some of them backwards. [isNew] lets each sample through
 * once.
 *
 * A stamp at or slightly before the last one is a repeat (false). A stamp more than [resetUs]
 * before the last one means the Loomo's clock stepped back; it is passed and becomes the new
 * reference, so a clock step cannot silence a source until the clock catches up again.
 */
class SampleGate(private val resetUs: Long = 1_000_000L) {
    private var lastUs = Long.MIN_VALUE

    /** Samples passed and repeats rejected since start, for the stats line. */
    @Volatile var passed = 0L
        private set
    @Volatile var repeats = 0L
        private set

    /** True if a sample stamped [stampUs] has not been seen yet; remembers it if so. */
    fun isNew(stampUs: Long): Boolean {
        val new = lastUs == Long.MIN_VALUE || stampUs > lastUs || stampUs < lastUs - resetUs
        if (new) {
            lastUs = stampUs
            passed++
        } else {
            repeats++
        }
        return new
    }
}
