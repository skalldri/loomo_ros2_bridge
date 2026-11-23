package com.autom8ed.lr2

import android.content.Context
import android.content.Intent
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
 *  - [TOPIC_DIAGNOSTICS] (diagnostic_msgs/DiagnosticArray), the standard topic, with three
 *    statuses built by [LoomoStatus]: the battery, the base's state and the app's own health.
 *  - [TOPIC_BATTERY] (sensor_msgs/BatteryState): the charge, and whether it is charging. The SDK
 *    gives nothing else (no voltage, current or temperature), so those fields are NaN.
 *
 * Sources:
 *  - the charge: Base.getRobotPower(), in percent, read only while Base is bound;
 *  - charging: Sensor.CHARGE_STATUS. If the query fails once it is not tried again until the
 *    app restarts, so a Loomo that does not implement it is not asked every second;
 *  - the base's state: the RobotState broadcasts of Segway's locomotion service
 *    (RobotEventStateManager). The service answers a QUERY_STATE broadcast, which is sent on
 *    every tick, so the state stays fresh even if the service only broadcasts changes.
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

    private val mBase = Base.getInstance()
    private val mSensor = Sensor.getInstance()
    private val mHead = Head.getInstance()
    private val mVersion = versionOf(ctx)

    private val mStateManager = RobotEventStateManager()
    @Volatile private var mBaseState: LoomoStatus.BaseState? = null
    @Volatile private var mBaseStateAtNs = 0L

    @Volatile private var mRunning = false
    private var mThread: Thread? = null
    private var mChargeStatusWorks = true
    private var mFailures = 0L

    fun start() {
        if (mRunning) return
        mRunning = true
        try {
            mStateManager.initialize(ctx, object : RobotEventStateManager.EventStateChangedListener {
                override fun onStateChanged(robotEventState: RobotEventState) {}

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
        queryBaseState()
        val percent = readPercent()
        val charging = readCharging()
        val nowNs = System.nanoTime()
        val statuses = listOf(
            LoomoStatus.battery(percent, charging),
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
        publishBattery(percent, charging)
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

    private fun readCharging(): Boolean? {
        if (!mChargeStatusWorks || !mSensor.isBind) return null
        return try {
            val data = mSensor.querySensorData(CHARGE_QUERY)
            data[0].intData[0] == 1
        } catch (t: Throwable) {
            mChargeStatusWorks = false
            Log.w(TAG, "Sensor.CHARGE_STATUS failed; not asking again until the app restarts", t)
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

    private fun publishBattery(percent: Int?, charging: Boolean?) {
        val msg = sensor_msgs.msg.BatteryState()
        stamp(msg.header)
        msg.voltage = Float.NaN
        msg.temperature = Float.NaN
        msg.current = Float.NaN
        msg.charge = Float.NaN
        msg.capacity = Float.NaN
        msg.designCapacity = Float.NaN
        msg.percentage = if (percent != null && percent in 0..100) percent / 100f else Float.NaN
        msg.powerSupplyStatus = when (charging) {
            true -> sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_CHARGING
            false -> sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
            null -> sensor_msgs.msg.BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        }
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
        private const val PERIOD_NS = 1_000_000_000L
        private const val LOCOMOTION_SERVICE_PACKAGE = "com.segway.robot.host.coreservice.locomotionservice"
        private val CHARGE_QUERY = listOf(Sensor.CHARGE_STATUS)

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
