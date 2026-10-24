package com.autom8ed.lr2

import com.autom8ed.lr2.DriveGate.Event
import com.autom8ed.lr2.DriveGate.Mode
import com.autom8ed.lr2.DriveGate.Reason
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

private const val MS = 1_000_000L

/** The base as the gate sees it: records every command, can refuse or throw. */
private class FakeWheels : DriveGate.Wheels {
    val sent = ArrayList<Pair<Double, Double>>()
    var accept = true
    var throwOnSet = false

    override fun set(linear: Double, angular: Double): Boolean {
        if (throwOnSet) throw RuntimeException("binder died")
        if (!accept) return false
        sent.add(linear to angular)
        return true
    }

    val last get() = sent.lastOrNull()
}

private class Rig {
    var nowNs = 1_000L * MS
    val wheels = FakeWheels()
    val gate = DriveGate(wheels, clockNs = { nowNs })
    val events = ArrayList<Event>()
    val statuses = ArrayList<DriveGate.Status>()

    init {
        gate.addListener(object : DriveGate.Listener {
            override fun onStatus(status: DriveGate.Status) {
                statuses.add(status)
            }

            override fun onEvent(event: Event) {
                events.add(event)
            }
        })
    }

    fun advance(ms: Long) {
        nowNs += ms * MS
    }

    /** One press and release on the button. */
    fun touch() {
        gate.touchDown()
        gate.tap()
    }

    /** Heartbeat, then a touch: driving, with no command sent yet. */
    fun startDriving() {
        gate.heartbeat()
        touch()
        assertTrue(gate.status().driving)
    }
}

class DriveGateTest {

    // --- Start-up and push to start ---

    @Test
    fun startsNotDrivingAndIgnoresCmdVel() {
        val r = Rig()
        assertEquals(Mode.NOT_DRIVING, r.gate.status().mode)
        assertEquals(Reason.START_UP, r.gate.status().reason)
        r.gate.heartbeat()
        r.gate.cmdVel(0.5, 0.2)
        r.advance(100)
        r.gate.tick()
        assertTrue(r.wheels.sent.isEmpty())
        assertFalse(r.gate.status().driving)
    }

    @Test
    fun aTapWithoutHeartbeatIsRefused() {
        val r = Rig()
        r.touch()
        assertFalse(r.gate.status().driving)
        assertEquals(Event.StartRefused(null), r.events.last())
        r.gate.cmdVel(0.5, 0.0)
        assertTrue(r.wheels.sent.isEmpty())
    }

    @Test
    fun aTapAfterTheHeartbeatHasLapsedIsRefused() {
        val r = Rig()
        r.gate.heartbeat()
        r.advance(1501)
        r.touch()
        assertFalse(r.gate.status().driving)
        assertEquals(Event.StartRefused(1501 * MS), r.events.last())
    }

    @Test
    fun aTapWithHeartbeatStartsDrivingAndCmdVelReachesTheWheels() {
        val r = Rig()
        r.startDriving()
        assertEquals(DriveGate.Status(Mode.DRIVING, null, heartbeatOk = true, following = false), r.gate.status())
        assertTrue(r.events.contains(Event.Started))
        assertTrue(r.wheels.sent.isEmpty()) // starting sends nothing until a command arrives
        r.gate.cmdVel(0.3, -0.1)
        assertEquals(0.3 to -0.1, r.wheels.last)
        assertTrue(r.gate.status().following)
    }

    @Test
    fun aCommandFromBeforeTheStartIsNotReplayed() {
        val r = Rig()
        r.gate.heartbeat()
        r.gate.cmdVel(1.0, 0.0) // while not driving
        r.touch()
        r.advance(10)
        r.gate.tick()
        assertTrue(r.wheels.sent.isEmpty())
        assertFalse(r.gate.status().following)
    }

    @Test
    fun pressingTheButtonWhileDrivingStopsOnThePressAndTheReleaseDoesNotRestart() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.5, 0.0)
        r.gate.touchDown()
        assertFalse(r.gate.status().driving)
        assertEquals(Reason.BUTTON, r.gate.status().reason)
        assertEquals(0.0 to 0.0, r.wheels.last)
        r.gate.tap() // the release of the same press
        assertFalse(r.gate.status().driving)
        r.gate.cmdVel(0.5, 0.0)
        assertEquals(0.0 to 0.0, r.wheels.last)
    }

    @Test
    fun aSecondTouchStartsAgain() {
        val r = Rig()
        r.startDriving()
        r.touch() // stop
        assertFalse(r.gate.status().driving)
        r.touch() // start
        assertTrue(r.gate.status().driving)
        assertNull(r.gate.status().reason)
    }

    @Test
    fun aCancelledStopPressDoesNotBlockTheNextStart() {
        val r = Rig()
        r.startDriving()
        r.gate.touchDown() // stops; the finger slides off, no tap
        assertFalse(r.gate.status().driving)
        r.touch()
        assertTrue(r.gate.status().driving)
    }

    @Test
    fun aPressWithoutReleaseDoesNotStart() {
        val r = Rig()
        r.gate.heartbeat()
        r.gate.touchDown()
        assertFalse(r.gate.status().driving)
    }

    // --- /cmd_vel timeout: 0.5 s ---

    @Test
    fun cmdVelTimeoutEdges() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.4, 0.1)
        val sent = r.wheels.sent.size
        r.advance(499)
        r.gate.heartbeat()
        r.gate.tick()
        r.advance(1)
        r.gate.tick() // exactly 0.5 s: not yet
        assertEquals(sent, r.wheels.sent.size)
        assertTrue(r.gate.status().following)
        r.advance(1)
        r.gate.tick() // 0.501 s
        assertEquals(sent + 1, r.wheels.sent.size)
        assertEquals(0.0 to 0.0, r.wheels.last)
        assertEquals(Event.CmdTimeout(501 * MS, 0.4, 0.1), r.events.last())
        assertTrue(r.gate.status().driving) // a timeout does not stop driving
        assertFalse(r.gate.status().following)
    }

    @Test
    fun theTimeoutZeroesOnceAndTheNextCommandMovesAgain() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.4, 0.0)
        r.advance(501)
        r.gate.heartbeat()
        r.gate.tick()
        r.advance(20)
        r.gate.tick()
        r.advance(20)
        r.gate.tick()
        assertEquals(listOf(0.4 to 0.0, 0.0 to 0.0), r.wheels.sent)
        assertEquals(1, r.events.count { it is Event.CmdTimeout })
        r.gate.cmdVel(0.2, 0.0)
        assertEquals(0.2 to 0.0, r.wheels.last)
        assertTrue(r.gate.status().following)
    }

    @Test
    fun noTimeoutBeforeTheFirstCommand() {
        val r = Rig()
        r.startDriving()
        for (i in 0 until 5) {
            r.advance(400)
            r.gate.heartbeat()
            r.gate.tick()
        }
        assertTrue(r.wheels.sent.isEmpty())
        assertTrue(r.events.none { it is Event.CmdTimeout })
    }

    @Test
    fun commandsEvery100MsNeverTimeOut() {
        val r = Rig()
        r.startDriving()
        for (i in 0 until 50) {
            r.gate.cmdVel(0.3, 0.0)
            r.advance(100)
            if (i % 5 == 0) r.gate.heartbeat()
            r.gate.tick()
        }
        assertTrue(r.events.none { it is Event.CmdTimeout })
        assertTrue(r.wheels.sent.all { it == 0.3 to 0.0 })
    }

    // --- Heartbeat: 1.5 s, latched ---

    @Test
    fun heartbeatTimeoutEdges() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.3, 0.0)
        // Keep the commands coming so only the heartbeat can stop it.
        for (i in 0 until 15) {
            r.advance(100)
            r.gate.cmdVel(0.3, 0.0)
        }
        r.gate.tick() // exactly 1.5 s after the heartbeat: still driving
        assertTrue(r.gate.status().driving)
        r.advance(1)
        r.gate.tick() // 1.501 s
        assertFalse(r.gate.status().driving)
        assertEquals(Reason.HEARTBEAT_LOST, r.gate.status().reason)
        assertFalse(r.gate.status().heartbeatOk)
        assertEquals(0.0 to 0.0, r.wheels.last)
        assertTrue(r.events.contains(Event.HeartbeatLost(1501 * MS)))
        assertTrue(r.events.contains(Event.Stopped(Reason.HEARTBEAT_LOST)))
    }

    @Test
    fun theHeartbeatStopIsLatchedUntilATouch() {
        val r = Rig()
        r.startDriving()
        r.advance(1501)
        r.gate.tick()
        assertFalse(r.gate.status().driving)
        r.gate.heartbeat() // the heartbeat comes back
        assertTrue(r.gate.status().heartbeatOk)
        assertTrue(r.events.contains(Event.HeartbeatBack))
        r.gate.cmdVel(0.5, 0.0)
        r.advance(1000)
        r.gate.heartbeat()
        r.gate.tick()
        assertFalse(r.gate.status().driving)
        assertEquals(Reason.HEARTBEAT_LOST, r.gate.status().reason)
        assertEquals(0.0 to 0.0, r.wheels.last)
        r.touch() // re-arm
        assertTrue(r.gate.status().driving)
        r.gate.cmdVel(0.5, 0.0)
        assertEquals(0.5 to 0.0, r.wheels.last)
    }

    @Test
    fun aLateHeartbeatStillStopsWithoutATickInBetween() {
        val r = Rig()
        r.startDriving()
        r.advance(1600)
        r.gate.heartbeat() // the gap ended with this heartbeat, but it was a gap
        assertFalse(r.gate.status().driving)
        assertEquals(Reason.HEARTBEAT_LOST, r.gate.status().reason)
    }

    @Test
    fun aCommandAfterTheHeartbeatLapsedIsRefusedBeforeTheNextTick() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.3, 0.0)
        r.advance(1501)
        r.gate.cmdVel(0.3, 0.0)
        assertFalse(r.gate.status().driving)
        assertEquals(listOf(0.3 to 0.0, 0.0 to 0.0), r.wheels.sent)
    }

    @Test
    fun heartbeatsEvery500MsKeepDriving() {
        val r = Rig()
        r.startDriving()
        for (i in 0 until 40) {
            r.advance(100)
            r.gate.cmdVel(0.1, 0.0)
            if (i % 5 == 4) r.gate.heartbeat()
            r.gate.tick()
        }
        assertTrue(r.gate.status().driving)
        assertTrue(r.events.none { it is Event.HeartbeatLost })
    }

    // --- The wheels ---

    @Test
    fun aRefusedZeroIsRetriedUntilTheBaseTakesIt() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.3, 0.0)
        r.wheels.accept = false
        r.gate.touchDown() // stop: the zero is refused
        assertEquals(0.3 to 0.0, r.wheels.last)
        r.advance(20)
        r.gate.tick()
        r.advance(20)
        r.gate.tick()
        assertEquals(1, r.events.count { it == Event.ZeroRefused }) // reported once
        r.wheels.accept = true
        r.advance(20)
        r.gate.tick()
        assertEquals(0.0 to 0.0, r.wheels.last)
        val n = r.wheels.sent.size
        r.advance(20)
        r.gate.tick()
        assertEquals(n, r.wheels.sent.size) // and not again once taken
    }

    @Test
    fun aThrowingBaseCountsAsRefusedAndTheGateKeepsWorking() {
        val r = Rig()
        r.startDriving()
        r.wheels.throwOnSet = true
        r.gate.cmdVel(0.3, 0.0)
        r.gate.touchDown()
        assertFalse(r.gate.status().driving)
        assertTrue(r.events.contains(Event.ZeroRefused))
        r.wheels.throwOnSet = false
        r.advance(20)
        r.gate.tick()
        assertEquals(0.0 to 0.0, r.wheels.last)
    }

    @Test
    fun aNonFiniteCommandSendsZero() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(Double.NaN, 0.2)
        assertEquals(0.0 to 0.0, r.wheels.last)
        r.gate.cmdVel(0.1, Double.POSITIVE_INFINITY)
        assertEquals(0.0 to 0.0, r.wheels.last)
        assertEquals(2, r.events.count { it is Event.InvalidCmd })
    }

    @Test
    fun baseBoundSendsZeroUnlessDriving() {
        val r = Rig()
        r.gate.baseBound()
        assertEquals(listOf(0.0 to 0.0), r.wheels.sent)
        r.startDriving()
        r.gate.cmdVel(0.2, 0.0)
        r.gate.baseBound()
        assertEquals(0.2 to 0.0, r.wheels.last)
    }

    @Test
    fun shutdownStopsAndZeroes() {
        val r = Rig()
        r.startDriving()
        r.gate.cmdVel(0.2, 0.0)
        r.gate.shutdown()
        assertFalse(r.gate.status().driving)
        assertEquals(Reason.APP_STOPPING, r.gate.status().reason)
        assertEquals(0.0 to 0.0, r.wheels.last)
    }

    // --- Listeners ---

    @Test
    fun statusIsReportedOncePerChange() {
        val r = Rig()
        r.gate.heartbeat()
        r.gate.heartbeat()
        r.gate.tick()
        r.touch()
        r.gate.cmdVel(0.1, 0.0)
        r.gate.cmdVel(0.1, 0.0)
        val expected = listOf(
            DriveGate.Status(Mode.NOT_DRIVING, Reason.START_UP, heartbeatOk = false, following = false),
            DriveGate.Status(Mode.NOT_DRIVING, Reason.START_UP, heartbeatOk = true, following = false),
            DriveGate.Status(Mode.DRIVING, null, heartbeatOk = true, following = false),
            DriveGate.Status(Mode.DRIVING, null, heartbeatOk = true, following = true),
        )
        assertEquals(expected, r.statuses)
    }

    @Test
    fun aThrowingListenerDoesNotStopTheStop() {
        val r = Rig()
        r.gate.addListener(object : DriveGate.Listener {
            override fun onEvent(event: Event) = throw IllegalStateException("listener bug")
            override fun onStatus(status: DriveGate.Status) = throw IllegalStateException("listener bug")
        })
        r.startDriving()
        r.gate.cmdVel(0.3, 0.0)
        r.gate.touchDown()
        assertFalse(r.gate.status().driving)
        assertEquals(0.0 to 0.0, r.wheels.last)
    }

    @Test
    fun statsLineCounts() {
        val r = Rig()
        r.gate.cmdVel(0.1, 0.0) // ignored
        r.startDriving()
        r.gate.cmdVel(0.1, 0.0)
        r.advance(501)
        r.gate.heartbeat()
        r.gate.tick()
        val line = r.gate.statsLine()
        assertTrue(line, line.contains("mode=DRIVING"))
        assertTrue(line, line.contains("passed=1 ignored=1"))
        assertTrue(line, line.contains("timeouts=1 starts=1"))
    }
}

class DriveStateRepeaterTest {
    @Test
    fun publishesOnEveryChangeAndOncePerPeriod() {
        val published = ArrayList<Pair<Long, Boolean>>()
        var now = 0L
        val rep = DriveStateRepeater(periodNs = 1000 * MS) { published.add(now to it) }
        for (t in 0L..2500L step 20L) {
            now = t * MS
            rep.update(driving = t in 1200L..1300L, nowNs = now)
        }
        assertEquals(
            listOf(
                0L * MS to false,
                1000L * MS to false,
                1200L * MS to true, // change
                1320L * MS to false, // change back
                2320L * MS to false,
            ),
            published
        )
    }
}
