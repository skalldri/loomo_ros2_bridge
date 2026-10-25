package com.autom8ed.lr2

import androidx.compose.foundation.background
import androidx.compose.foundation.gestures.detectTapGestures
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.rememberUpdatedState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.input.pointer.pointerInput
import androidx.compose.ui.platform.LocalDensity
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.TextUnit
import androidx.compose.ui.unit.dp

private val DrivingBackground = Color(0xFF0B3D12)
private val StoppedBackground = Color(0xFF161616)
private val DrivingText = Color(0xFF5CFF6A)
private val WarningText = Color(0xFFFFB300)
private val EStopRed = Color(0xFFD50000)
private val EStopYellow = Color(0xFFFFD600)

/**
 * The Loomo's screen: the push-to-start button, styled as an emergency stop (red on a yellow
 * ring), and the driving state in large letters. The button stops driving as soon as it is
 * pressed; a tap (press and release on the button) starts it.
 */
@Composable
fun DriveScreen(
    status: DriveGate.Status,
    onTouchDown: () -> Unit,
    onTap: () -> Unit,
    modifier: Modifier = Modifier,
) {
    // A sleeping screen takes the first touch to wake up: keep it on while driving, so a single
    // touch always stops.
    val view = LocalView.current
    DisposableEffect(status.driving) {
        view.keepScreenOn = status.driving
        onDispose { }
    }

    BoxWithConstraints(
        modifier = modifier
            .fillMaxSize()
            .background(if (status.driving) DrivingBackground else StoppedBackground)
            .padding(16.dp),
        contentAlignment = Alignment.Center,
    ) {
        val landscape = maxWidth > maxHeight
        val shortSide = minOf(maxWidth, maxHeight)
        val density = LocalDensity.current
        val titleSize = with(density) { (shortSide * 0.13f).toSp() }
        val detailSize = with(density) { (shortSide * 0.055f).toSp() }
        val labelSize = with(density) { (shortSide * 0.11f).toSp() }
        if (landscape) {
            val buttonSize = minOf(maxHeight * 0.9f, maxWidth * 0.45f)
            // Two fixed halves, so the button stays put while the state text changes width.
            Row(
                modifier = Modifier.fillMaxSize(),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Box(Modifier.weight(1f), contentAlignment = Alignment.Center) {
                    EStopButton(status.driving, buttonSize, labelSize, onTouchDown, onTap)
                }
                Box(Modifier.weight(1f), contentAlignment = Alignment.Center) {
                    StateText(status, titleSize, detailSize)
                }
            }
        } else {
            val buttonSize = minOf(maxWidth * 0.85f, maxHeight * 0.55f)
            Column(
                modifier = Modifier.fillMaxSize(),
                verticalArrangement = Arrangement.SpaceEvenly,
                horizontalAlignment = Alignment.CenterHorizontally,
            ) {
                StateText(status, titleSize, detailSize)
                EStopButton(status.driving, buttonSize, labelSize, onTouchDown, onTap)
            }
        }
    }
}

@Composable
private fun StateText(status: DriveGate.Status, titleSize: TextUnit, detailSize: TextUnit) {
    Column(horizontalAlignment = Alignment.CenterHorizontally) {
        Text(
            text = if (status.driving) "DRIVING" else "NOT DRIVING",
            color = if (status.driving) DrivingText else Color.White,
            fontSize = titleSize,
            fontWeight = FontWeight.Bold,
            textAlign = TextAlign.Center,
        )
        Text(
            text = driveDetail(status),
            color = if (status.heartbeatOk) Color(0xFFE0E0E0) else WarningText,
            fontSize = detailSize,
            textAlign = TextAlign.Center,
        )
    }
}

@Composable
private fun EStopButton(
    driving: Boolean,
    size: Dp,
    labelSize: TextUnit,
    onTouchDown: () -> Unit,
    onTap: () -> Unit,
) {
    val down by rememberUpdatedState(onTouchDown)
    val tap by rememberUpdatedState(onTap)
    Box(
        modifier = Modifier
            .size(size)
            .clip(CircleShape)
            .background(EStopYellow)
            .pointerInput(Unit) {
                detectTapGestures(
                    onPress = {
                        down()
                        tryAwaitRelease()
                    },
                    onTap = { tap() },
                )
            }
            .padding(size * 0.09f)
            .clip(CircleShape)
            .background(EStopRed),
        contentAlignment = Alignment.Center,
    ) {
        Text(
            text = if (driving) "STOP" else "START",
            color = Color.White,
            fontSize = labelSize,
            fontWeight = FontWeight.Black,
        )
    }
}

/** The line under the state: what the wheels do, or why the app is not driving. */
fun driveDetail(status: DriveGate.Status): String = when {
    status.driving && status.following -> "Following /cmd_vel\nTouch to stop"
    status.driving -> "Wheels at zero: no /cmd_vel\nTouch to stop"
    status.reason == DriveGate.Reason.HEARTBEAT_LOST && !status.heartbeatOk ->
        "Stopped: Jetson heartbeat lost\nTouch to restart once it is back"
    status.reason == DriveGate.Reason.HEARTBEAT_LOST -> "Stopped: Jetson heartbeat was lost\nTouch to restart"
    !status.heartbeatOk -> "No Jetson heartbeat: cannot start"
    status.reason == DriveGate.Reason.BUTTON -> "Stopped by the button\nTouch to start"
    status.reason == DriveGate.Reason.UI_HIDDEN -> "Stopped: the app left the screen\nTouch to start"
    status.reason == DriveGate.Reason.APP_STOPPING -> "The app is stopping"
    else -> "Touch to start"
}
