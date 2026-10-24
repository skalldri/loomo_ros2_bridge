package com.autom8ed.lr2

import android.os.Bundle
import android.os.Handler
import android.util.Log
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.produceState
import androidx.compose.ui.Modifier
import com.autom8ed.lr2.ui.theme.LoomoROS2BridgeTheme
import kotlinx.coroutines.CoroutineExceptionHandler
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.launch
import org.ros2.android.activity.ROSActivity
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.executors.Executor
import org.ros2.rcljava.executors.MultiThreadedExecutor
import java.util.Timer
import java.util.concurrent.Executors
import java.util.concurrent.ScheduledExecutorService
import java.util.concurrent.TimeUnit
import com.autom8ed.lr2.vision.CameraInterface
import com.autom8ed.lr2.vision.FrameStamp


class MainActivity : ComponentActivity() {

    // private lateinit var mVisionInterface: VisionInterface
    private lateinit var mCameraInterface: CameraInterface
    private lateinit var mLocomotionPlatformInterface: LocomotionPlatformInterface
    private lateinit var mSensorInterface: SensorInterface
    private lateinit var mTfPublisher: TfPublisher
    private lateinit var mHeadInterface: HeadInterface
    private lateinit var mAudioInterface: AudioInterface
    private lateinit var mWatchdog: Watchdog
    private lateinit var mStatsReporter: StatsReporter
    private lateinit var mNetworkGate: NetworkGate

    // Push to start and the stop path (DriveGate). Created before ROS, so the screen works and the
    // wheels stay at zero while the app waits for the network.
    private lateinit var mDriveGate: DriveGate
    private lateinit var mDriveStatus: MutableStateFlow<DriveGate.Status>
    private var mDriveTicker: ScheduledExecutorService? = null
    // Set once ROS is up; read by the ticker thread.
    @Volatile private var mDriveStatePublisher: LocomotionPlatformInterface? = null

    // The Ethernet link's speed as the Jetson reports it; the screen warns while it is slow.
    private val mJetsonLink = JetsonLink()

    private lateinit var mNode: RosNode

    private var rosExecutor: Executor? = null
    private var timer: Timer? = null
    private var handler: Handler? = null

    private val TAG: String = ROSActivity::class.java.name

    private val SPINNER_DELAY: Long = 0
    private val SPINNER_PERIOD_MS: Long = 10

    private var isWorking = false

    @OptIn(DelicateCoroutinesApi::class)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        handler = Handler(mainLooper)

        // Which streams and transports run, from bridge.properties and the intent extras. Loaded
        // before RCLJava.rclJavaInit() because dds.priority decides the Fast DDS profile below.
        val config = BridgeConfig.load(this, intent)
        config.log()

        // Must be called before RCLJava.rclJavaInit() to override the automatic DDS/RTPS discovery
        // protocol!
        // Desktop
        // android.system.Os.setenv("ROS_DISCOVERY_SERVER", "192.168.1.36:11811", true)
        // Jetson
        // android.system.Os.setenv("ROS_DISCOVERY_SERVER", "10.0.0.1:11811", true)
        // android.system.Os.setenv("ROS_STATIC_PEERS", "192.168.1.174", true)


        // Set ROS_DOMAIN_ID so that the robot runs on an isolated DDS network
        // Other nodes will not attempt to talk directly with the robot, instead they must go through
        // the DDS-Router on the Jetson, which bridges domain 1 <-> 0 with a topic allowlist and
        // republishes the H.264 stream RELIABLE for the Isaac ROS decoder (the decoder cannot be
        // told to subscribe BEST_EFFORT). Measured 2026-10-05 on the Jetson Orin Nano: the router
        // costs ~9% of one core carrying ~10 MB/s of raw images plus odom to domain-0 consumers.
        android.system.Os.setenv("ROS_DOMAIN_ID", "1", true)

        // TODO: does not work on Android
        // Set FastDDS into "large data" mode, to help it transmit data faster
        // android.system.Os.setenv("ROS_STATIC_PEERS", "10.0.0.1", true)

        android.system.Os.setenv("FASTDDS_BUILTIN_TRANSPORTS", "DEFAULT", true)

        // Asynchronous publishing: rcl_publish() returns once the sample is in the writer
        // history and Fast DDS's sender thread does the fragmentation and the UDP sends. In the
        // default synchronous mode the publishing thread did the ~10-20 sendto() calls of a
        // 614 KB depth frame itself (12-23 ms per frame measured on the frame workers, with three
        // streams contending for the one link). rmw_fastrtps reads this variable for every
        // DataWriter. Set to SYNCHRONOUS to compare.
        android.system.Os.setenv("RMW_FASTRTPS_PUBLICATION_MODE", "ASYNCHRONOUS", true)

        //android.system.Os.setenv("FASTDDS_BUILTIN_TRANSPORTS", "LARGE_DATA", true)

        // With one asynchronous sender, TF, odometry and joint states queued behind ~12 MB/s of
        // raw images and almost never left the app (loomo_ros2_bridge#17). The profile gives
        // them priority over the images.
        if (config.ddsPriority) {
            FastDdsProfile.install(this)?.let {
                android.system.Os.setenv("FASTRTPS_DEFAULT_PROFILES_FILE", it, true)
            }
        }

        startDriveGate()

        enableEdgeToEdge()
        setContent {
            LoomoROS2BridgeTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    val status by mDriveStatus.collectAsState()
                    // Polled, so the banner also goes away when the reports stop.
                    val linkWarning by produceState<String?>(null) {
                        while (true) {
                            value = mJetsonLink.warning(System.nanoTime())
                            delay(LINK_POLL_MS)
                        }
                    }
                    DriveScreen(
                        status = status,
                        onTouchDown = { mDriveGate.touchDown() },
                        onTap = { mDriveGate.tap() },
                        modifier = Modifier.padding(innerPadding),
                        linkWarning = linkWarning,
                    )
                }
            }
        }

        // ROS (and with it the Fast DDS participant) starts only once the Ethernet link to the
        // Jetson has an address, and the app restarts if that network is lost or replaced; see
        // NetworkGate for why.
        mNetworkGate = NetworkGate(
            this,
            restartExtras = intent?.extras,
            beforeRestart = { stopDriving(); stopStreams() },
            onReady = { network -> startRos(config, network) },
        )
        mNetworkGate.start()
    }

    /**
     * The gate starts not driving. Its ticker applies the /cmd_vel and heartbeat timeouts every
     * DriveGate.TICK_MS and publishes the driving state once ROS is up.
     */
    private fun startDriveGate() {
        mDriveGate = DriveGate(BaseWheels())
        mDriveStatus = MutableStateFlow(mDriveGate.status())
        mDriveGate.addListener(object : DriveGate.Listener {
            override fun onStatus(status: DriveGate.Status) {
                mDriveStatus.value = status
            }

            override fun onEvent(event: DriveGate.Event) {
                logDriveEvent(event)
            }
        })
        Log.i(DRIVE_TAG, "not driving until the button is touched; /cmd_vel timeout " +
            "${mDriveGate.cmdTimeoutNs / 1_000_000} ms, heartbeat timeout ${mDriveGate.heartbeatTimeoutNs / 1_000_000} ms")
        val ticker = Executors.newSingleThreadScheduledExecutor { r ->
            Thread(r, "drive-gate").apply { isDaemon = true }
        }
        ticker.scheduleWithFixedDelay({
            // An exception would cancel the schedule: never let one escape.
            try {
                mDriveGate.tick()
                mDriveStatePublisher?.publishDriveState(mDriveGate.status().driving, System.nanoTime())
            } catch (t: Throwable) {
                Log.e(DRIVE_TAG, "drive gate tick failed", t)
            }
        }, DriveGate.TICK_MS, DriveGate.TICK_MS, TimeUnit.MILLISECONDS)
        mDriveTicker = ticker
    }

    private fun logDriveEvent(event: DriveGate.Event) {
        fun ms(ns: Long?) = if (ns == null) "never" else "${ns / 1_000_000} ms"
        when (event) {
            is DriveGate.Event.Started -> Log.w(DRIVE_TAG, "DRIVING: /cmd_vel now reaches the wheels (button)")
            is DriveGate.Event.StartRefused ->
                Log.w(DRIVE_TAG, "start refused: no Jetson heartbeat (last: ${ms(event.heartbeatAgeNs)})")
            is DriveGate.Event.StartRefusedHidden ->
                Log.w(DRIVE_TAG, "start refused: the screen is not in front")
            is DriveGate.Event.Stopped -> Log.w(DRIVE_TAG, "NOT DRIVING (${event.reason}): wheels set to zero")
            is DriveGate.Event.CmdTimeout -> Log.w(
                DRIVE_TAG, "/cmd_vel timeout: none for ${ms(event.ageNs)}; wheels set to zero " +
                    "(last linear=${event.linear} angular=${event.angular})"
            )
            is DriveGate.Event.HeartbeatLost ->
                Log.e(DRIVE_TAG, "Jetson heartbeat missing for ${ms(event.ageNs)}")
            is DriveGate.Event.HeartbeatBack -> Log.i(DRIVE_TAG, "Jetson heartbeat received")
            is DriveGate.Event.ZeroRefused ->
                Log.e(DRIVE_TAG, "the base did not take a zero; retrying every ${DriveGate.TICK_MS} ms")
            is DriveGate.Event.InvalidCmd -> Log.e(
                DRIVE_TAG, "/cmd_vel with a non-finite value (linear=${event.linear} " +
                    "angular=${event.angular}): wheels set to zero"
            )
        }
    }

    private fun stopDriving() {
        if (this::mDriveGate.isInitialized) {
            mDriveGate.shutdown()
        }
    }

    @OptIn(DelicateCoroutinesApi::class)
    private fun startRos(config: BridgeConfig, network: android.net.Network) {
        // Clock offset to the Jetson, so every stamp is on the Jetson's clock. Started first: the
        // first answer normally arrives while the rest is still being set up, and stamped
        // messages are held back until it does (see TimeSync).
        TimeSync.start(network, config.clockServer, config.clockSync)

        RCLJava.rclJavaInit()
        rosExecutor = this.createExecutor()

        mNode = RosNode("loomo_node")
        changeState(true);

        mHeadInterface = HeadInterface(this, mNode)

        mAudioInterface = AudioInterface(this, mNode)

        mSensorInterface = SensorInterface(this, mNode)

        mTfPublisher = TfPublisher(this, mNode, mSensorInterface, config)

        mLocomotionPlatformInterface = LocomotionPlatformInterface(this, mNode, mDriveGate)
        mDriveStatePublisher = mLocomotionPlatformInterface

        FrameStamp.TAG_FRAME_NUM = config.tagFrameNum

        // Delivery accounting for the camera streams: one logcat line per stream every few
        // seconds under the tag "StreamStats", plus an error line for every dropped or failed frame.
        mStatsReporter = StatsReporter(config.statsPeriodS * 1000L)
        mStatsReporter.registerExtra { mTfPublisher.statsLine() }
        mStatsReporter.registerExtra { mDriveGate.statsLine() }
        mStatsReporter.registerExtra { mJetsonLink.statsLine(System.nanoTime()) }
        mStatsReporter.start()

        mCameraInterface = CameraInterface(this, mNode, mTfPublisher, mStatsReporter, config)

        // A camera failure must not take the app down with it (and the drive gate with it).
        GlobalScope.launch(CoroutineExceptionHandler { _, e -> Log.e(TAG, "camera start failed", e) }) {
            mCameraInterface.start()
        }

        //mVisionInterface = VisionInterface(this, mNode, mTfPublisher, TimeSync)

        /**/

        /*
        val list = MediaCodecList(MediaCodecList.REGULAR_CODECS)
        for (cd in list.codecInfos) {
            Log.i(TAG, "Found Codec: ${cd.name}")
        }
        */

        mWatchdog = Watchdog(this, mNode, mHeadInterface) { mDriveGate.heartbeat() }

        // The Jetson reads the link's speed (the Loomo cannot) and sends it every second.
        mNode.node.createSubscription(std_msgs.msg.UInt16::class.java, JetsonLink.TOPIC) { msg ->
            val mbps = JetsonLink.mbpsOf(msg.data)
            if (mJetsonLink.onReport(mbps, System.nanoTime())) {
                if (mbps in 1 until mJetsonLink.fullMbps) {
                    Log.w(LINK_TAG, "Ethernet to the Jetson at $mbps Mb/s, below ${mJetsonLink.fullMbps}: showing the warning")
                } else {
                    Log.i(LINK_TAG, "Ethernet to the Jetson: ${if (mbps == 0) "no carrier" else "$mbps Mb/s"} (Jetson's report)")
                }
            }
        }

        // Should not block since we are using multithreading
        // should also run as fast as possible, always executing new work as it arrives
        rosExecutor!!.spin()
    }

    override fun onDestroy() {
        super.onDestroy()
        stopDriving()
        mDriveTicker?.shutdownNow()
        stopStreams()
    }

    private fun stopStreams() {
        if (this::mCameraInterface.isInitialized) {
            mCameraInterface.stop()
        }
        if (this::mStatsReporter.isInitialized) {
            mStatsReporter.stop()
        }
    }

    override fun onResume() {
        super.onResume()
        // The button can be reached again; driving stays off until it is tapped.
        if (this::mDriveGate.isInitialized) mDriveGate.uiShown()
        /*
        timer = Timer()
        timer!!.schedule(object : TimerTask() {
            override fun run() {
                val runnable = Runnable { rosExecutor!!.spinSome() }
                handler!!.post(runnable)
            }
        }, this.getDelay(), this.getPeriod())
         */
    }

    override fun onPause() {
        super.onPause()
        // Another app, a system dialog or the screen turning off: the button can no longer be
        // reached, so stop driving (latched). While driving the screen is kept on, so it cannot
        // time out into this; the power button still can, and stops.
        if (this::mDriveGate.isInitialized) mDriveGate.uiHidden()
        timer?.cancel()
    }

    private fun changeState(isWorking: Boolean) {
        this.isWorking = isWorking
        //val buttonStart = findViewById<View>(R.id.buttonStart) as Button
        //val buttonStop = findViewById<View>(R.id.buttonStop) as Button
        //buttonStart.isEnabled = !isWorking
        //buttonStop.isEnabled = isWorking
        if (isWorking) {
            getExecutor()!!.addNode(mNode)
            mNode.start()
        } else {
            mNode.stop()
            getExecutor()!!.removeNode(mNode)
        }
    }

    fun getExecutor(): Executor? {
        return this.rosExecutor
    }

    protected fun createExecutor(): Executor {
        return MultiThreadedExecutor()
    }

    protected fun getDelay(): Long {
        return SPINNER_DELAY
    }

    protected fun getPeriod(): Long {
        return SPINNER_PERIOD_MS
    }
}

private const val DRIVE_TAG = "DriveGate"
private const val LINK_TAG = "JetsonLink"
private const val LINK_POLL_MS = 500L
