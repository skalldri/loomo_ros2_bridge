package com.autom8ed.loomoros2bridge

import android.R
import android.os.Bundle
import android.os.Handler
import android.view.View
import android.widget.Button
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.tooling.preview.Preview
import com.autom8ed.loomoros2bridge.ui.theme.LoomoROS2BridgeTheme
import kotlinx.coroutines.DelicateCoroutinesApi
import kotlinx.coroutines.GlobalScope
import kotlinx.coroutines.launch
import org.ros2.android.activity.ROSActivity
import org.ros2.rcljava.RCLJava
import org.ros2.rcljava.executors.Executor
import org.ros2.rcljava.executors.SingleThreadedExecutor
import java.util.Timer
import java.util.TimerTask


class MainActivity : ComponentActivity() {

    private lateinit var mVisionInterface: VisionInterface;
    private lateinit var mLocomotionPlatformInterface: LocomotionPlatformInterface;

    private lateinit var mNode: RosNode

    private var rosExecutor: Executor? = null
    private var timer: Timer? = null
    private var handler: Handler? = null

    private val logtag: String = ROSActivity::class.java.name

    private val SPINNER_DELAY: Long = 0
    private val SPINNER_PERIOD_MS: Long = 200

    private var isWorking = false

    @OptIn(DelicateCoroutinesApi::class)
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        handler = Handler(mainLooper)

        // Must be called before RCLJava.rclJavaInit() to override the automatic DDS/RTPS discovery
        // protocol!
        android.system.Os.setenv("ROS_DISCOVERY_SERVER", "192.168.1.36:11811", true)

        RCLJava.rclJavaInit()
        rosExecutor = this.createExecutor()

        enableEdgeToEdge()
        setContent {
            LoomoROS2BridgeTheme {
                Scaffold(modifier = Modifier.fillMaxSize()) { innerPadding ->
                    Greeting(
                        name = "Android",
                        modifier = Modifier.padding(innerPadding)
                    )
                }
            }
        }

        mNode = RosNode("loomo_node")
        changeState(true);

        mLocomotionPlatformInterface = LocomotionPlatformInterface(this, mNode)

        mVisionInterface = VisionInterface(this, mNode)

        GlobalScope.launch {
            //mVisionInterface.startCameraStream(Camera.DEPTH, "realsense_depth")
            mVisionInterface.startCameraStream(Camera.FISH_EYE, "fisheye")
        }

        //    mVisionInterface.startCameraStream(Camera.FISH_EYE, "fisheye")
        //    mVisionInterface.startCameraStream(Camera.COLOR, "realsense_color")
        // }
    }

    override fun onResume() {
        super.onResume()
        timer = Timer()
        timer!!.schedule(object : TimerTask() {
            override fun run() {
                val runnable = Runnable { rosExecutor!!.spinSome() }
                handler!!.post(runnable)
            }
        }, this.getDelay(), this.getPeriod())
    }

    override fun onPause() {
        super.onPause()
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
        return SingleThreadedExecutor()
    }

    protected fun getDelay(): Long {
        return SPINNER_DELAY
    }

    protected fun getPeriod(): Long {
        return SPINNER_PERIOD_MS
    }
}

@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(
        text = "Hello $name!",
        modifier = modifier
    )
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    LoomoROS2BridgeTheme {
        Greeting("Android")
    }
}