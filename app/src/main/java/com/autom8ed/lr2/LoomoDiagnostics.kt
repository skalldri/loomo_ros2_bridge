package com.autom8ed.lr2

import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.os.BatteryManager
import android.util.Log
import com.segway.robot.sdk.base.action.RobotAction
import com.segway.robot.sdk.base.state.RobotEventState
import com.segway.robot.sdk.base.state.RobotEventStateManager
import com.segway.robot.sdk.base.state.RobotState
import com.segway.robot.sdk.locomotion.sbv.Base
import com.segway.robot.sdk.perception.sensor.Sensor
import org.ros2.rcljava.publisher.Publisher

/**
 * Publishes the Loomo's health once a second. It only reads: nothing here commands the robot.
 *  - [TOPIC_DIAGNOSTICS] (diagnostic_msgs/DiagnosticArray), the standard topic, with four
 *    statuses built by [LoomoStatus]: the base's battery, the head's battery, the base's state
 *    and the app's own health.
 *  - [TOPIC_BATTERY] (sensor_msgs/BatteryState): the charge. The SDK gives nothing else (no
 *    voltage, current, temperature or charging state), so those fields are NaN or unknown.
 *  - [TOPIC_HEAD_BATTERY] (sensor_msgs/BatteryState): the head's own Li-ion cell, from Android
 *    (percentage, voltage, temperature, status, health).
 *
 * Sources:
 *  - the charge: Base.getRobotPower(), in percent, read only while Base is bound;
 *  - nothing for charging. Sensor.CHARGE_STATUS returned an empty list on our Loomo both on
 *    battery and on the charger, and Segway's BATTERY_CHANGED broadcast carries only the charge
 *    (2026-11-24);
 *  - the base's state: the RobotState broadcasts of Segway's locomotion service
 *    (RobotEventStateManager). The service broadcasts events (PUSHING, LIFT_UP, ...) when they
 *    happen, and a RobotState in answer to QUERY_STATE. Each event triggers a query, so a change
 *    shows within about a second (a push on 2026-11-24 did). Without events, a query goes out
 *    when the state is older than [QUERY_AFTER_NS], at most every [QUERY_PERIOD_NS]: the service
 *    logs a warning line for each one, and logcat wraps within minutes.
 *
 * Messages are stamped on the Jetson's clock once [TimeSync] is ready, and with zero before
 * that: diagnostics are published even when the clock sync is down, since that is when they
 * matter.
 */
class LoomoDiagnostics(
    private val ctx: Context,
    node: RosNode,
    private val jetsonLink: JetsonLink,
) {
    private val mDiagnosticsPublisher: Publisher<diagnostic_msgs.msg.DiagnosticArray> =
        node.node.createPublisher(diagnostic_msgs.msg.DiagnosticArray::class.java, TOPIC_DIAGNOSTICS)
    private val mBatteryPublisher: Publisher<sensor_msgs.msg.BatteryState> =
        node.node.createPublisher(sensor_msgs.msg.BatteryState::class.java, TOPIC_BATTERY)
    private val mHeadBatteryPublisher: Publisher<sensor_msgs.msg.BatteryState> =
        node.node.createPublisher(sensor_msgs.msg.BatteryState::class.java, TOPIC_HEAD_BATTERY)

    private val mBase = Base.getInstance()
    private val mSensor = Sensor.getInstance()
    private val mHead = Head.getInstance()
    private val mVersion = versionOf(ctx)

    private val mStateManager = RobotEventStateManager()
    @Volatile private var mBaseState: LoomoStatus.BaseState? = null
    @Volatile private var mBaseStateAtNs = 0L
    @Volatile private var mEventPending = false
    private var mLastQueryNs = 0L

    @Volatile private var mRunning = false
    private var mThread: Thread? = null
    private var mFailures = 0L

    fun start() {
        if (mRunning) return
        mRunning = true
        try {
            mStateManager.initialize(ctx, object : RobotEventStateManager.EventStateChangedListener {
                override fun onStateChanged(robotEventState: RobotEventState) {
                    mEventPending = true
                }

                override fun onRobotStateUpdated(robotState: RobotState) {
                    mBaseState = baseStateOf(robotState)
                    mBaseStateAtNs = System.nanoTime()
                }
            }, *RobotAction.ROBOT_ACTIONS)
        } catch (t: Throwable) {
            Log.e(TAG, "could not register for the base's state", t)
        }
        mThread = Thread({ loop() }, "loomo-diagnostics").apply { isDaemon = true; start() }
    }

    fun stop() {
        mRunning = false
        mThread?.interrupt()
        try {
            mStateManager.release()
        } catch (t: Throwable) {
            Log.w(TAG, "releasing the state manager failed", t)
        }
    }

    private fun loop() {
        val rate = FixedRate(PERIOD_NS, System.nanoTime())
        while (mRunning) {
            val wait = rate.waitNs(System.nanoTime())
            if (wait > 0) {
                try {
                    Thread.sleep(wait / 1_000_000L, (wait % 1_000_000L).toInt())
                } catch (e: InterruptedException) {
                    return
                }
            }
            if (!rate.due(System.nanoTime())) continue
            try {
                tick()
            } catch (t: Throwable) {
                // A persistent failure: log the first, then once every 10 minutes.
                val n = ++mFailures
                if (n == 1L || n % 600 == 0L) Log.e(TAG, "diagnostics failed ($n so far)", t)
            }
        }
    }

    private fun tick() {
        val queryNs = System.nanoTime()
        if (mEventPending || mLastQueryNs == 0L ||
            (queryNs - mBaseStateAtNs > QUERY_AFTER_NS && queryNs - mLastQueryNs >= QUERY_PERIOD_NS)
        ) {
            mEventPending = false
            mLastQueryNs = queryNs
            queryBaseState()
        }
        val percent = readPercent()
        val head = readHeadBattery()
        val nowNs = System.nanoTime()
        val statuses = listOf(
            LoomoStatus.battery(percent),
            LoomoStatus.headBattery(head),
            LoomoStatus.baseState(mBaseState, nowNs - mBaseStateAtNs),
            LoomoStatus.app(
                LoomoStatus.AppHealth(
                    baseBound = mBase.isBind,
                    sensorBound = mSensor.isBind,
                    headBound = mHead.isBind,
                    timeSynced = TimeSync.isReady(),
                    clockOffsetNs = TimeSync.offsetNs,
                    link = jetsonLink.status(nowNs),
                    version = mVersion,
                )
            ),
        )
        publishDiagnostics(statuses)
        publishBattery(percent)
        publishHeadBattery(head)
    }

    /** The head's own battery: Android's sticky ACTION_BATTERY_CHANGED (no receiver is kept). */
    private fun readHeadBattery(): LoomoStatus.HeadBattery? {
        val i = ctx.registerReceiver(null, IntentFilter(Intent.ACTION_BATTERY_CHANGED)) ?: return null
        val level = i.getIntExtra(BatteryManager.EXTRA_LEVEL, -1)
        val scale = i.getIntExtra(BatteryManager.EXTRA_SCALE, -1)
        if (level < 0 || scale <= 0) return null
        return LoomoStatus.HeadBattery(
            percent = level * 100 / scale,
            voltageMv = i.getIntExtra(BatteryManager.EXTRA_VOLTAGE, 0),
            temperatureDeciC = i.getIntExtra(BatteryManager.EXTRA_TEMPERATURE, 0),
            health = i.getIntExtra(BatteryManager.EXTRA_HEALTH, BatteryManager.BATTERY_HEALTH_UNKNOWN),
            status = i.getIntExtra(BatteryManager.EXTRA_STATUS, BatteryManager.BATTERY_STATUS_UNKNOWN),
            plugged = i.getIntExtra(BatteryManager.EXTRA_PLUGGED, 0),
        )
    }

    private fun publishHeadBattery(b: LoomoStatus.HeadBattery?) {
        val msg = sensor_msgs.msg.BatteryState()
        stamp(msg.header)
        msg.current = Float.NaN
        msg.charge = Float.NaN
        msg.capacity = Float.NaN
        msg.designCapacity = Float.NaN
        msg.present = b != null
        msg.location = "head"
        msg.powerSupplyTechnology = sensor_msgs.msg.BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        if (b == null) {
            msg.voltage = Float.NaN
            msg.temperature = Float.NaN
            msg.percentage = Float.NaN
        } else {
            msg.voltage = b.voltageMv / 1000f
            msg.temperature = b.temperatureDeciC / 10f
            msg.percentage = b.percent / 100f
            // BatteryState's codes are Android's minus one (both follow Linux's power_supply).
            msg.powerSupplyStatus = (b.status - 1).coerceIn(0, 4).toByte()
            msg.powerSupplyHealth = (b.health - 1).coerceIn(0, 6).toByte()
        }
        mHeadBatteryPublisher.publish(msg)
    }

    private fun readPercent(): Int? {
        if (!mBase.isBind) return null
        return try {
            mBase.robotPower
        } catch (t: Throwable) {
            Log.w(TAG, "Base.getRobotPower failed", t)
            null
        }
    }

    /** Asks Segway's locomotion service for its RobotState broadcast. */
    private fun queryBaseState() {
        val intent = Intent(RobotAction.RobotEvent.QUERY_STATE)
        intent.setPackage(LOCOMOTION_SERVICE_PACKAGE)
        intent.putExtra(RobotAction.ExtraKey.KEY_PACKAGE_NAME, ctx.packageName)
        ctx.sendBroadcast(intent)
    }

    private fun stamp(header: std_msgs.msg.Header) {
        if (TimeSync.isReady()) TimeSync.toJetson(header.stamp, TimeSync.nowLoomoUs())
    }

    private fun publishDiagnostics(statuses: List<LoomoStatus.Status>) {
        val msg = diagnostic_msgs.msg.DiagnosticArray()
        stamp(msg.header)
        msg.status = statuses.map { s ->
            diagnostic_msgs.msg.DiagnosticStatus()
                .setLevel(s.level)
                .setName(s.name)
                .setMessage(s.message)
                .setHardwareId(LoomoStatus.HARDWARE_ID)
                .also { d ->
                    d.values = s.values.map { (k, v) -> diagnostic_msgs.msg.KeyValue().setKey(k).setValue(v) }
                }
        }
        mDiagnosticsPublisher.publish(msg)
    }

    private fun publishBattery(percent: Int?) {
        val msg = sensor_msgs.msg.BatteryState()
        stamp(msg.header)
        msg.voltage = Float.NaN
        msg.temperature = Float.NaN
        msg.current = Float.NaN
        msg.charge = Float.NaN
        msg.capacity = Float.NaN
        msg.designCapacity = Float.NaN
        msg.percentage = if (percent != null && percent in 0..100) percent / 100f else Float.NaN
        msg.powerSupplyStatus = sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        msg.powerSupplyHealth = sensor_msgs.msg.BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.powerSupplyTechnology = sensor_msgs.msg.BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = percent != null
        msg.location = "base"
        mBatteryPublisher.publish(msg)
    }

    companion object {
        private const val TAG = "LoomoDiagnostics"
        const val TOPIC_DIAGNOSTICS = "/diagnostics"
        const val TOPIC_BATTERY = "/loomo/battery"
        /** Not under /loomo/head/: that is the head's command namespace. */
        const val TOPIC_HEAD_BATTERY = "/loomo/head_battery"
        private const val PERIOD_NS = 1_000_000_000L
        /** Ask for the base's state when the last one is older than this... */
        private const val QUERY_AFTER_NS = 4_000_000_000L
        /** ...but at most this often, unless an event arrived. */
        private const val QUERY_PERIOD_NS = 5_000_000_000L
        private const val LOCOMOTION_SERVICE_PACKAGE = "com.segway.robot.host.coreservice.locomotionservice"

        fun baseStateOf(s: RobotState) = LoomoStatus.BaseState(
            fallen = s.isFallDown,
            stuck = s.isBaseStuck,
            spinning = s.isBaseSpinning,
            lifted = s.isLift,
            pushing = s.isPushing,
            locked = s.isBaseLocked,
            steppedOn = s.isStep,
            rider = s.isRider,
            standUp = s.isStandUp,
            pitchLock = s.isPitchLock,
            yawLock = s.isYawLock,
            mode = s.mode,
        )

        private fun versionOf(ctx: Context): String = try {
            val info = ctx.packageManager.getPackageInfo(ctx.packageName, 0)
            // versionName is always 1.0: the install time tells builds apart.
            val installed = java.text.SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'", java.util.Locale.ROOT)
                .apply { timeZone = java.util.TimeZone.getTimeZone("UTC") }
                .format(java.util.Date(info.lastUpdateTime))
            "${info.versionName}, installed $installed"
        } catch (t: Throwable) {
            "unknown"
        }
    }
}
