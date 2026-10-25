package com.autom8ed.lr2

/**
 * At most one ROS stack per process.
 *
 * MainActivity's ROS stack (node, executor, TfPublisher and SensorInterface threads, TimeSync)
 * outlives the activity: onDestroy stops only the cameras and the stats. So if a second
 * MainActivity starts ROS in the same process, everything is published twice. Seen on the robot
 * on 2026-10-23, when a relaunch intent finished one MainActivity and created another in the same
 * process: odometry, the base tilt and the head IMU went out twice, with the same stamps.
 *
 * The first instance to [claim] the guard owns the stack for the life of the process; it is never
 * released, because the stack keeps running after its activity is destroyed. Any other instance
 * restarts the app instead (NetworkGate.restart). The guard lives in process memory, so a fresh
 * process always starts ROS normally and the restart cannot loop.
 *
 * Plain Kotlin, so the logic is unit tested; MainActivity holds one instance per process.
 */
class RosStackGuard {
    private var owner: Any? = null
    private var ownerStop: (() -> Unit)? = null

    /**
     * Claims the stack for [instance]. True if it may start ROS: nobody owned it (now [instance]
     * does, and [stop] is kept) or [instance] already owns it. False if another instance owns it.
     */
    @Synchronized
    fun claim(instance: Any, stop: () -> Unit): Boolean {
        val current = owner
        if (current == null) {
            owner = instance
            ownerStop = stop
            return true
        }
        return current === instance
    }

    /** True if an instance other than [instance] owns the stack. */
    @Synchronized
    fun ownedByOther(instance: Any): Boolean {
        val current = owner
        return current != null && current !== instance
    }

    /**
     * Runs the owner's stop (driving and streams) when [instance] is not the owner, before
     * [instance] restarts the process. Returns true if it ran.
     */
    fun stopOwnerFor(instance: Any): Boolean {
        val stop = synchronized(this) {
            if (owner == null || owner === instance) null else ownerStop
        } ?: return false
        stop()
        return true
    }
}
