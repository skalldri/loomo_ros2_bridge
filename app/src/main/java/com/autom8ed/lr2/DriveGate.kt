package com.autom8ed.lr2

import java.util.concurrent.CopyOnWriteArrayList

/**
 * Push to start and the stop path: decides whether `/cmd_vel` reaches the wheels.
 *
 * - The app starts NOT_DRIVING. Only a tap on the screen's button switches to DRIVING; no topic
 *   can. A tap cannot start driving while the Jetson's heartbeat is missing.
 * - While DRIVING, every `/cmd_vel` goes to the wheels.
 * - The wheels are set to zero:
 *   - at once when the button is pressed while driving (on the press, not the release);
 *   - when no `/cmd_vel` has arrived for [cmdTimeoutNs] (0.5 s): a stopped teleop or planner, a
 *     lost link. Driving stays on, and the next `/cmd_vel` moves the wheels again;
 *   - when no heartbeat (`/loomo/jetson_watchdog`, sent at 2 Hz) has arrived for
 *     [heartbeatTimeoutNs] (1.5 s). The gate drops to NOT_DRIVING and stays there (latched) until
 *     the button is tapped again, even if the heartbeat comes back.
 * - NOT_DRIVING only ever sends zeros. The base stays in raw mode and keeps balancing at zero
 *   speed; nothing here changes its control mode or turns balancing off.
 *
 * Every entry point first applies the timeouts at the current time, so a `/cmd_vel` that arrives
 * after a missed heartbeat is refused even before the next [tick]. The methods are synchronized
 * and command the wheels under the same lock, so a `/cmd_vel` racing a stop can never reach the
 * wheels after the stop's zero. A zero the base does not take (service not bound, binder error)
 * is retried on every call until it does.
 *
 * Plain Kotlin with an injected clock and wheels, so JVM unit tests cover it (DriveGateTest).
 */
class DriveGate(
    private val wheels: Wheels,
    private val clockNs: () -> Long = System::nanoTime,
    val cmdTimeoutNs: Long = CMD_TIMEOUT_NS,
    val heartbeatTimeoutNs: Long = HEARTBEAT_TIMEOUT_NS,
) {
    fun interface Wheels {
        /** Commands the base. False (or an exception) if the base did not take the command. */
        fun set(linear: Double, angular: Double): Boolean
    }

    enum class Mode { NOT_DRIVING, DRIVING }

    /** Why the gate is NOT_DRIVING. */
    enum class Reason { START_UP, BUTTON, HEARTBEAT_LOST, APP_STOPPING }

    data class Status(
        val mode: Mode,
        /** Why the gate is not driving; null while driving. */
        val reason: Reason?,
        /** A heartbeat arrived within the last [heartbeatTimeoutNs]. */
        val heartbeatOk: Boolean,
        /** Driving and the wheels follow `/cmd_vel`. False until the first command after a
         *  start, and after a `/cmd_vel` timeout until the next command. */
        val following: Boolean,
    ) {
        val driving: Boolean get() = mode == Mode.DRIVING
    }

    sealed class Event {
        data object Started : Event()
        /** A tap while the heartbeat was missing; [heartbeatAgeNs] is null if none ever came. */
        data class StartRefused(val heartbeatAgeNs: Long?) : Event()
        data class Stopped(val reason: Reason) : Event()
        /** No `/cmd_vel` for [ageNs]; the wheels were set to zero from [linear], [angular]. */
        data class CmdTimeout(val ageNs: Long, val linear: Double, val angular: Double) : Event()
        data class HeartbeatLost(val ageNs: Long?) : Event()
        data object HeartbeatBack : Event()
        /** The base did not take a zero; it is retried until it does. Reported once per episode. */
        data object ZeroRefused : Event()
        /** A `/cmd_vel` with a NaN or infinite field; the wheels got zero instead. */
        data class InvalidCmd(val linear: Double, val angular: Double) : Event()
    }

    interface Listener {
        /** Called with the new status after every change, under the gate's lock. */
        fun onStatus(status: Status) {}
        /** Called for every event, under the gate's lock. */
        fun onEvent(event: Event) {}
    }

    private val listeners = CopyOnWriteArrayList<Listener>()

    private var mode = Mode.NOT_DRIVING
    private var reason: Reason? = Reason.START_UP
    private var lastHeartbeatNs: Long? = null
    private var heartbeatOk = false
    private var lastCmdNs = 0L
    private var lastLinear = 0.0
    private var lastAngular = 0.0
    private var following = false
    private var zeroPending = false
    private var pressStopped = false
    private var notified: Status? = null

    // Counters for the stats line.
    private var cmdPassed = 0L
    private var cmdIgnored = 0L
    private var cmdInvalid = 0L
    private var cmdTimeouts = 0L
    private var starts = 0L
    private var startsRefused = 0L
    private var heartbeatStops = 0L
    private var buttonStops = 0L
    private var zeroRefusals = 0L

    @Synchronized
    fun status(): Status = Status(mode, reason, heartbeatOk, following)

    /** Adds [listener] and gives it the current status at once. */
    @Synchronized
    fun addListener(listener: Listener) {
        listeners.add(listener)
        try {
            listener.onStatus(status())
        } catch (t: Throwable) {
            // A listener must not break the gate.
        }
    }

    /** The button went down. Stops driving at once; an emergency stop acts on the press. */
    @Synchronized
    fun touchDown() {
        evaluate(clockNs())
        pressStopped = mode == Mode.DRIVING
        if (pressStopped) {
            buttonStops++
            stop(Reason.BUTTON)
        }
        notifyStatus()
    }

    /**
     * The button was tapped (released on the button after [touchDown]). Starts driving, unless the
     * same press just stopped it or the heartbeat is missing.
     */
    @Synchronized
    fun tap() {
        val now = clockNs()
        evaluate(now)
        if (pressStopped) {
            pressStopped = false
        } else if (mode != Mode.DRIVING) {
            if (heartbeatOk) {
                mode = Mode.DRIVING
                reason = null
                following = false
                starts++
                emit(Event.Started)
            } else {
                startsRefused++
                emit(Event.StartRefused(lastHeartbeatNs?.let { now - it }))
            }
        }
        notifyStatus()
    }

    /** A `/cmd_vel` message. It reaches the wheels only while driving. */
    @Synchronized
    fun cmdVel(linear: Double, angular: Double) {
        val now = clockNs()
        evaluate(now)
        lastCmdNs = now
        if (mode != Mode.DRIVING) {
            cmdIgnored++
            notifyStatus()
            return
        }
        var l = linear
        var a = angular
        if (!l.isFinite() || !a.isFinite()) {
            cmdInvalid++
            emit(Event.InvalidCmd(l, a))
            l = 0.0
            a = 0.0
        }
        lastLinear = l
        lastAngular = a
        if (send(l, a)) zeroPending = false
        cmdPassed++
        following = true
        notifyStatus()
    }

    /** A heartbeat from the Jetson. It never starts driving. */
    @Synchronized
    fun heartbeat() {
        val now = clockNs()
        // A gap that ended with this heartbeat still stops driving: apply the timeouts first.
        evaluate(now)
        lastHeartbeatNs = now
        if (!heartbeatOk) {
            heartbeatOk = true
            emit(Event.HeartbeatBack)
        }
        notifyStatus()
    }

    /** Applies the timeouts; the app calls this every [TICK_MS]. */
    @Synchronized
    fun tick() {
        evaluate(clockNs())
        notifyStatus()
    }

    /** The base service (re)bound. Unless driving, send it a zero in case it still moves. */
    @Synchronized
    fun baseBound() {
        if (mode != Mode.DRIVING) zeroWheels()
        notifyStatus()
    }

    /** The app is going away: stop driving and zero the wheels. */
    @Synchronized
    fun shutdown() {
        if (mode == Mode.DRIVING) stop(Reason.APP_STOPPING) else zeroWheels()
        notifyStatus()
    }

    /** One line for StatsReporter. */
    @Synchronized
    fun statsLine(): String =
        "drive mode=$mode reason=${reason ?: "-"} heartbeat=${if (heartbeatOk) "ok" else "missing"}" +
            " following=$following cmd passed=$cmdPassed ignored=$cmdIgnored invalid=$cmdInvalid" +
            " timeouts=$cmdTimeouts starts=$starts refused=$startsRefused" +
            " stops button=$buttonStops heartbeat=$heartbeatStops zero_refused=$zeroRefusals"

    private fun evaluate(now: Long) {
        val hbAge = lastHeartbeatNs?.let { now - it }
        val hbFresh = hbAge != null && hbAge <= heartbeatTimeoutNs
        if (heartbeatOk && !hbFresh) {
            heartbeatOk = false
            emit(Event.HeartbeatLost(hbAge))
        }
        if (mode == Mode.DRIVING && !hbFresh) {
            heartbeatStops++
            stop(Reason.HEARTBEAT_LOST)
        }
        if (mode == Mode.DRIVING && following) {
            val cmdAge = now - lastCmdNs
            if (cmdAge > cmdTimeoutNs) {
                following = false
                cmdTimeouts++
                zeroWheels()
                emit(Event.CmdTimeout(cmdAge, lastLinear, lastAngular))
            }
        }
        if (zeroPending) zeroWheels()
    }

    private fun stop(r: Reason) {
        mode = Mode.NOT_DRIVING
        reason = r
        following = false
        zeroWheels()
        emit(Event.Stopped(r))
    }

    private fun zeroWheels() {
        if (send(0.0, 0.0)) {
            zeroPending = false
        } else {
            if (!zeroPending) {
                zeroRefusals++
                emit(Event.ZeroRefused)
            }
            zeroPending = true
        }
    }

    private fun send(linear: Double, angular: Double): Boolean =
        try {
            wheels.set(linear, angular)
        } catch (t: Throwable) {
            false
        }

    private fun emit(event: Event) {
        for (l in listeners) {
            try {
                l.onEvent(event)
            } catch (t: Throwable) {
                // A listener must not break the gate.
            }
        }
    }

    private fun notifyStatus() {
        val s = status()
        if (s == notified) return
        notified = s
        for (l in listeners) {
            try {
                l.onStatus(s)
            } catch (t: Throwable) {
                // A listener must not break the gate.
            }
        }
    }

    companion object {
        const val CMD_TIMEOUT_NS = 500_000_000L
        const val HEARTBEAT_TIMEOUT_NS = 1_500_000_000L
        /** How often the app calls [tick]: a timeout acts at most this late. */
        const val TICK_MS = 20L
    }
}

/**
 * Publishes the gate's driving state through [publish]: at once on every change, and again every
 * [periodNs] so that readers which cannot keep a transient-local sample (or join through a bridge
 * that drops durability) still see it within a period, and can tell a live app from a stale one.
 */
class DriveStateRepeater(
    private val periodNs: Long = 1_000_000_000L,
    private val publish: (Boolean) -> Unit,
) {
    private var last: Boolean? = null
    private var lastAtNs = 0L

    /** Call often (every gate tick) with the current state. */
    fun update(driving: Boolean, nowNs: Long) {
        if (driving != last || nowNs - lastAtNs >= periodNs) {
            publish(driving)
            last = driving
            lastAtNs = nowNs
        }
    }
}
