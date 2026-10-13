package com.autom8ed.lr2

import android.app.AlarmManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.net.ConnectivityManager
import android.net.LinkProperties
import android.net.Network
import android.net.NetworkCapabilities
import android.net.NetworkRequest
import android.os.Bundle
import android.os.Handler
import android.os.Process
import android.os.SystemClock
import android.util.Log
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
 * All decisions run on the main thread; [onReady] is called there at most once.
 */
class NetworkGate(
    private val ctx: Context,
    private val restartExtras: Bundle?,
    private val beforeRestart: () -> Unit,
    private val onReady: () -> Unit,
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
            onReady()
        } else if (network != current) {
            restart("Ethernet network replaced ($current -> $network)")
        }
    }

    private fun ipv4Addresses(network: Network): List<String> =
        cm.getLinkProperties(network)?.linkAddresses
            ?.filter { it.address is Inet4Address }
            ?.map { it.toString() }
            ?: emptyList()

    private fun restart(reason: String) {
        if (restarting) return
        restarting = true
        Log.e(TAG, "$reason: restarting the app so DDS gets sockets on the current network")
        try {
            beforeRestart()
        } catch (t: Throwable) {
            Log.e(TAG, "cleanup before restart failed", t)
        }
        val intent = Intent(ctx, MainActivity::class.java)
            .addFlags(Intent.FLAG_ACTIVITY_NEW_TASK or Intent.FLAG_ACTIVITY_CLEAR_TASK)
        restartExtras?.let { intent.putExtras(it) }
        val pending = PendingIntent.getActivity(ctx, 0, intent, PendingIntent.FLAG_CANCEL_CURRENT)
        val alarms = ctx.getSystemService(Context.ALARM_SERVICE) as AlarmManager
        alarms.set(AlarmManager.ELAPSED_REALTIME, SystemClock.elapsedRealtime() + RESTART_DELAY_MS, pending)
        Process.killProcess(Process.myPid())
    }

    companion object {
        const val TAG = "NetworkGate"
        private const val RESTART_DELAY_MS = 1500L
    }
}
