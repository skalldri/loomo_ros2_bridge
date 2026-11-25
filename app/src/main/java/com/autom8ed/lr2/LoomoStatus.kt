package com.autom8ed.lr2

/**
 * The Loomo's diagnostic statuses, as plain values: [LoomoDiagnostics] turns them into
 * diagnostic_msgs/DiagnosticStatus. Kept free of the SDK and of ROS so it can be unit-tested.
 *
 * Every name starts with [PREFIX], so the Jetson's diagnostic aggregator can group them.
 */
object LoomoStatus {
    // diagnostic_msgs/DiagnosticStatus levels.
    const val OK: Byte = 0
    const val WARN: Byte = 1
    const val ERROR: Byte = 2
    const val STALE: Byte = 3

    const val PREFIX = "loomo: "
    const val BATTERY = PREFIX + "Battery"
    const val BASE_STATE = PREFIX + "Base state"
    const val APP = PREFIX + "App"
    const val HEAD_BATTERY = PREFIX + "Head battery"
    const val HARDWARE_ID = "loomo"

    /** Below this charge (percent) the battery status is WARN. */
    const val BATTERY_WARN_PERCENT = 20
    /** Below this charge (percent) the battery status is ERROR. */
    const val BATTERY_ERROR_PERCENT = 10

    /** At or above this temperature (degrees C) the head battery is WARN. */
    const val HEAD_BATTERY_WARN_C = 45.0

    /** A base state older than this (ns) is STALE: Segway's service stopped answering. */
    const val BASE_STATE_STALE_NS = 5_000_000_000L

    data class Status(
        val name: String,
        val level: Byte,
        val message: String,
        val values: List<Pair<String, String>> = emptyList(),
    )

    /** The base's battery, from Base.getRobotPower() ([percent], null if Base is unbound or the read failed). */
    fun battery(percent: Int?): Status {
        if (percent == null) {
            return Status(BATTERY, STALE, "no reading from the base")
        }
        if (percent !in 0..100) {
            return Status(BATTERY, STALE, "invalid reading: $percent %", listOf("percent" to percent.toString()))
        }
        val level = when {
            percent < BATTERY_ERROR_PERCENT -> ERROR
            percent < BATTERY_WARN_PERCENT -> WARN
            else -> OK
        }
        val low = when (level) {
            ERROR -> " (critical)"
            WARN -> " (low)"
            else -> ""
        }
        return Status(BATTERY, level, "$percent %$low", listOf("percent" to percent.toString()))
    }

    /** The base's state as Segway's locomotion service reports it (sdkbase RobotState). */
    data class BaseState(
        val fallen: Boolean = false,
        val stuck: Boolean = false,
        val spinning: Boolean = false,
        val lifted: Boolean = false,
        val pushing: Boolean = false,
        val locked: Boolean = false,
        val steppedOn: Boolean = false,
        val rider: Boolean = false,
        val standUp: Boolean = false,
        val pitchLock: Boolean = false,
        val yawLock: Boolean = false,
        /** RobotState.MODE_ROBOT (0), MODE_SBV (1, self-balancing vehicle) or -1 (unknown). */
        val mode: Int = -1,
    )

    /**
     * The base's state. [state] is the last one received, null if none yet; [ageNs] is how long
     * ago it arrived.
     */
    fun baseState(state: BaseState?, ageNs: Long): Status {
        if (state == null) {
            return Status(BASE_STATE, STALE, "no state from Segway's service yet")
        }
        val values = listOf(
            "mode" to modeName(state.mode),
            "fallen" to state.fallen.toString(),
            "stuck" to state.stuck.toString(),
            "spinning" to state.spinning.toString(),
            "lifted" to state.lifted.toString(),
            "pushing" to state.pushing.toString(),
            "locked" to state.locked.toString(),
            "stepped_on" to state.steppedOn.toString(),
            "rider" to state.rider.toString(),
            "stand_up" to state.standUp.toString(),
            "pitch_lock" to state.pitchLock.toString(),
            "yaw_lock" to state.yawLock.toString(),
            "age_s" to String.format(java.util.Locale.ROOT, "%.1f", ageNs / 1e9),
        )
        if (ageNs > BASE_STATE_STALE_NS) {
            return Status(BASE_STATE, STALE, "no update from Segway's service for ${ageNs / 1_000_000_000L} s", values)
        }
        val errors = listOfNotNull(
            "fallen".takeIf { state.fallen },
            "stuck".takeIf { state.stuck },
        )
        val warnings = listOfNotNull(
            "lifted".takeIf { state.lifted },
            "pushed".takeIf { state.pushing },
            "locked".takeIf { state.locked },
        )
        val level = when {
            errors.isNotEmpty() -> ERROR
            warnings.isNotEmpty() -> WARN
            else -> OK
        }
        val message = if (level == OK) "normal" else (errors + warnings).joinToString(", ")
        return Status(BASE_STATE, level, message, values)
    }

    fun modeName(mode: Int): String = when (mode) {
        0 -> "robot"
        1 -> "self-balancing vehicle"
        else -> "unknown"
    }

    /** What the app knows about its own health. */
    data class AppHealth(
        val baseBound: Boolean,
        val sensorBound: Boolean,
        val headBound: Boolean,
        val timeSynced: Boolean,
        val clockOffsetNs: Long,
        val link: JetsonLink.Status,
        val version: String,
    )

    fun app(h: AppHealth): Status {
        val values = listOf(
            "base_bound" to h.baseBound.toString(),
            "sensor_bound" to h.sensorBound.toString(),
            "head_bound" to h.headBound.toString(),
            "time_synced" to h.timeSynced.toString(),
            "clock_offset_ms" to String.format(java.util.Locale.ROOT, "%.3f", h.clockOffsetNs / 1e6),
            "jetson_link" to h.link.state.name.lowercase(),
            "jetson_link_mbps" to (h.link.mbps?.toString() ?: "unknown"),
            "version" to h.version,
        )
        val errors = listOfNotNull("base unbound".takeIf { !h.baseBound })
        val warnings = listOfNotNull(
            "sensor unbound".takeIf { !h.sensorBound },
            "head unbound".takeIf { !h.headBound },
            "clock not synced".takeIf { !h.timeSynced },
            "Ethernet at ${h.link.mbps} Mb/s".takeIf { h.link.state == JetsonLink.State.SLOW },
            "no link report from the Jetson".takeIf { h.link.state == JetsonLink.State.UNKNOWN },
        )
        val level = when {
            errors.isNotEmpty() -> ERROR
            warnings.isNotEmpty() -> WARN
            else -> OK
        }
        val message = if (level == OK) "running" else (errors + warnings).joinToString(", ")
        return Status(APP, level, message, values)
    }

    /**
     * The head's own battery, as Android reports it (ACTION_BATTERY_CHANGED): one Li-ion cell
     * (max170xx fuel gauge, bq24192 charger) that the base keeps charged. Not the base's battery,
     * which only Base.getRobotPower() reports.
     */
    data class HeadBattery(
        val percent: Int,
        val voltageMv: Int,
        val temperatureDeciC: Int,
        /** android.os.BatteryManager.BATTERY_HEALTH_* */
        val health: Int,
        /** android.os.BatteryManager.BATTERY_STATUS_* */
        val status: Int,
        /** android.os.BatteryManager.BATTERY_PLUGGED_*, 0 on battery */
        val plugged: Int,
    )

    // android.os.BatteryManager's values, so this stays testable on the JVM.
    const val ANDROID_HEALTH_GOOD = 2
    const val ANDROID_HEALTH_OVERHEAT = 3
    const val ANDROID_HEALTH_DEAD = 4
    const val ANDROID_HEALTH_OVER_VOLTAGE = 5
    const val ANDROID_HEALTH_UNSPECIFIED_FAILURE = 6
    const val ANDROID_HEALTH_COLD = 7

    fun androidHealthName(health: Int): String = when (health) {
        ANDROID_HEALTH_GOOD -> "good"
        ANDROID_HEALTH_OVERHEAT -> "overheat"
        ANDROID_HEALTH_DEAD -> "dead"
        ANDROID_HEALTH_OVER_VOLTAGE -> "over voltage"
        ANDROID_HEALTH_UNSPECIFIED_FAILURE -> "failure"
        ANDROID_HEALTH_COLD -> "cold"
        else -> "unknown"
    }

    fun androidStatusName(status: Int): String = when (status) {
        2 -> "charging"
        3 -> "discharging"
        4 -> "not charging"
        5 -> "full"
        else -> "unknown"
    }

    fun headBattery(b: HeadBattery?): Status {
        if (b == null) return Status(HEAD_BATTERY, STALE, "no reading from Android")
        val tempC = b.temperatureDeciC / 10.0
        val values = listOf(
            "percent" to b.percent.toString(),
            "voltage_v" to String.format(java.util.Locale.ROOT, "%.3f", b.voltageMv / 1000.0),
            "temperature_c" to String.format(java.util.Locale.ROOT, "%.1f", tempC),
            "health" to androidHealthName(b.health),
            "status" to androidStatusName(b.status),
            "external_power" to (b.plugged != 0).toString(),
        )
        val errors = listOfNotNull(
            "health ${androidHealthName(b.health)}".takeIf {
                b.health in listOf(
                    ANDROID_HEALTH_OVERHEAT, ANDROID_HEALTH_DEAD,
                    ANDROID_HEALTH_OVER_VOLTAGE, ANDROID_HEALTH_UNSPECIFIED_FAILURE,
                )
            },
            "${b.percent} % (critical)".takeIf { b.percent < BATTERY_ERROR_PERCENT },
        )
        val warnings = listOfNotNull(
            "health cold".takeIf { b.health == ANDROID_HEALTH_COLD },
            "${b.percent} % (low)".takeIf { b.percent in BATTERY_ERROR_PERCENT until BATTERY_WARN_PERCENT },
            String.format(java.util.Locale.ROOT, "%.1f °C", tempC).takeIf { tempC >= HEAD_BATTERY_WARN_C },
            "no external power".takeIf { b.plugged == 0 },
        )
        val level = when {
            errors.isNotEmpty() -> ERROR
            warnings.isNotEmpty() -> WARN
            else -> OK
        }
        val message = if (level == OK) "${b.percent} %, ${androidStatusName(b.status)}"
        else (errors + warnings).joinToString(", ")
        return Status(HEAD_BATTERY, level, message, values)
    }
}
