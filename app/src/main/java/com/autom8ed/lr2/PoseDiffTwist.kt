package com.autom8ed.lr2

import java.util.ArrayDeque
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

/**
 * Odometry twist from differences of the SDK's Pose2D, kept free of Android so it can be unit tested.
 *
 * The SDK's own Pose2D linearVelocity/angularVelocity are low-pass filtered (time constant ~0.45 s,
 * ~20 ms delay): against the L2 LIDAR's gyro on a driven circuit (2026-10-13) the SDK's angular
 * velocity lagged 395 ms with an RMS error of 0.062 rad/s. The pose itself is not filtered but
 * quantised (yaw in encoder ticks of 0.0192666 rad, translation in 4.45 mm steps), so a difference
 * over one 50 ms sample is mostly quantisation noise. Over a window of 0.25-0.30 s the difference
 * lagged 160-190 ms with an RMS error of 0.049-0.050 rad/s; 0.10-0.15 s windows were worse again
 * (one yaw tick is 0.13-0.19 rad/s there).
 *
 * [add] takes each unique Pose2D sample (raw Loomo stamp, microseconds) and returns the twist
 * between it and the newest earlier sample at least [windowUs] older, or null when the caller
 * should use the SDK's values: until the window has filled, and when that sample is more than
 * [maxDtUs] old (the Pose2D stream stalled; a difference over a long gap is an average, not a
 * velocity). The window is chosen by actual stamps, so a 100 ms Pose2D interval (~2% of them)
 * just makes that one window 50 ms longer.
 */
class PoseDiffTwist(
    private val windowUs: Long = 250_000L,
    private val maxDtUs: Long = 600_000L,
) {
    /** Linear (m/s, along the robot's heading) and angular (rad/s) velocity. */
    data class Twist(val linear: Double, val angular: Double)

    private class Sample(val tUs: Long, val x: Double, val y: Double, val theta: Double)

    private val samples = ArrayDeque<Sample>()

    fun add(tUs: Long, x: Double, y: Double, theta: Double): Twist? {
        val last = samples.peekLast()
        if (last != null && tUs <= last.tUs) {
            // A repeat (the caller dedupes; this is a guard) or the Loomo's clock stepped back:
            // differences across a step are meaningless, so start over.
            if (tUs == last.tUs) return null
            samples.clear()
        }
        samples.addLast(Sample(tUs, x, y, theta))

        // Keep exactly one sample at least a window old: the reference.
        while (samples.size >= 2 && tUs - secondOldest().tUs >= windowUs) samples.removeFirst()
        val ref = samples.peekFirst()!!
        val dtUs = tUs - ref.tUs
        if (dtUs < windowUs || dtUs > maxDtUs) return null

        val dt = dtUs / 1e6
        val dTheta = wrap(theta - ref.theta)
        // Project the displacement on the mean heading over the window: on an arc this is the
        // chord's direction, so a turning robot's forward speed is not shortened by cos(dTheta / 2).
        val thetaMid = ref.theta + dTheta / 2
        val v = ((x - ref.x) * cos(thetaMid) + (y - ref.y) * sin(thetaMid)) / dt
        return Twist(v, dTheta / dt)
    }

    private fun secondOldest(): Sample {
        val it = samples.iterator()
        it.next()
        return it.next()
    }

    companion object {
        /** [a] wrapped to (-pi, pi]. */
        fun wrap(a: Double): Double {
            var r = a % (2 * PI)
            if (r > PI) r -= 2 * PI
            if (r <= -PI) r += 2 * PI
            return r
        }
    }
}
