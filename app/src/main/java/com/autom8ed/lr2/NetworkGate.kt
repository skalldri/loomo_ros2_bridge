package com.autom8ed.lr2

import android.app.Activity
import android.content.Context
import android.content.Intent
import android.net.ConnectivityManager
import android.net.LinkProperties
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Bundle
import android.os.Handler
import android.util.Log
import com.jakewharton.processphoenix.ProcessPhoenix
import java.net.Inet4Address

/**
 * Starts ROS only once the Ethernet link to the Jetson has an IPv4 address, and restarts the app
 * when that network goes away or is replaced (loomo_ros2_bridge#20).
 *
 * Why both are needed:
 *  - Fast DDS enumerates network interfaces once, when the participant is created. The app is
 *    started by BootReceiver ~20 s before Android brings Ethernet up (the Loomo's address comes from
 *    the Jetson's DHCP server), so a participant created at boot only had loopback and never talked
 *    to the Jetson: the head LEDs stayed on the white "starting" animation (measured 2026-10-12).
 *  - When the link drops, Android's ConnectivityService destroys the Ethernet network and creates a
 *    new one, even with the same 10.0.0.2. Sockets created on the old network keep its fwmark, which
 *    then has no routing rule, so every Fast DDS send fails (Loomo eth0 TX was 0 pkt/s while the app
 *    "published" at 30 fps). The DDS objects cannot be rebuilt in place, so the process restarts and
 *    the new process waits for the network again.
 *
 * All decisions run on the main thread; [onReady] is called there at most once, with the network.
 */
class NetworkGate(
    private val ctx: Context,
    private val restartExtras: Bundle?,
    private val beforeRestart: () -> Unit,
    private val onReady: (Network) -> Unit,
) {
    private val cm = ctx.getSystemService(Context.CONNECTIVITY_SERVICE) as ConnectivityManager
    private val main = Handler(ctx.mainLooper)

    /** The Ethernet network ROS was started on; null while waiting. Only touched on the main thread. */
    private var rosNetwork: Network? = null
    private var restarting = false

    private val callback = object : ConnectivityManager.NetworkCallback() {
        override fun onAvailable(network: Network) {
            main.post { evaluate(network, "available") }
        }

        override fun onLinkPropertiesChanged(network: Network, linkProperties: LinkProperties) {
            main.post { evaluate(network, "link properties changed") }
        }

        override fun onLost(network: Network) {
            main.post {
                if (network == rosNetwork) {
                    restart("Ethernet network $network lost")
                } else {
                    Log.i(TAG, "Ethernet network $network lost (ROS not started on it)")
                }
            }
        }
    }

    fun start() {
        val request = NetworkRequest.Builder()
            .addTransportType(NetworkCapabilities.TRANSPORT_ETHERNET)
            // The Jetson link need not offer internet access.
            .removeCapability(NetworkCapabilities.NET_CAPABILITY_INTERNET)
            .build()
        Log.i(TAG, "waiting for an Ethernet network with an IPv4 address before starting ROS")
        // Delivers onAvailable for a network that is already up, so no separate initial check.
        cm.registerNetworkCallback(request, callback)
    }

    private fun evaluate(network: Network, why: String) {
        if (restarting) return
        val current = rosNetwork
        if (current == null) {
            val ipv4 = ipv4Addresses(network)
            if (ipv4.isEmpty()) {
                Log.i(TAG, "Ethernet network $network $why, no IPv4 address yet")
                return
            }
            rosNetwork = network
            Log.i(TAG, "Ethernet network $network up with $ipv4: starting ROS")
            onReady(network)
        } else if (network != current) {
            restart("Ethernet network replaced ($current -> $network)")
        }
    }

    private fun ipv4Addresses(network: Network): List<String> =
        cm.getLinkProperties(network)?.linkAddresses
            ?.filter { it.address is Inet4Address }
            ?.map { it.toString() }
            ?: emptyList()

    /** Restarts the app in a fresh process: [beforeRestart], then MainActivity is relaunched at once.
     *  Also used by MainActivity when Segway's SDK has unbound its services in the background, and
     *  when a second MainActivity finds ROS already running in this process. */
    fun restart(reason: String) {
        if (restarting) return
        restarting = true
        Log.e(TAG, "$reason: restarting the app")
        try {
            beforeRestart()
        } catch (t: Throwable) {
            Log.e(TAG, "cleanup before restart failed", t)
        }
        val intent = Intent(ctx, MainActivity::class.java)
        restartExtras?.let { intent.putExtras(it) }
        // ProcessPhoenix starts a small activity in a separate process (":phoenix"). It kills this
        // process, starts [intent] with NEW_TASK|CLEAR_TASK, and exits; MainActivity then comes up
        // in a fresh process. This replaces an AlarmManager relaunch, which Android 5.1 delays to
        // at least 5 s (4.95-4.99 s measured): the launcher showed for 5 s, and a tap on the icon
        // in that time put a second MainActivity in the relaunched process.
        ProcessPhoenix.triggerRebirth(ctx, intent)
        // Finish the activity before the process dies. An activity that has saved its state (it
        // went to the background) and dies while resuming is restarted by Android at once in a new
        // process; the relaunch then adds a second MainActivity to that process, and both run ROS.
        // The phoenix activity has its own task (AndroidManifest.xml), so this does not remove it.
        (ctx as? Activity)?.finishAndRemoveTask()
    }

    companion object {
        const val TAG = "NetworkGate"
    }
}
