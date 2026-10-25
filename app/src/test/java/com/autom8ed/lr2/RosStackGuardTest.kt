package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test

class RosStackGuardTest {
    private val a = Any()
    private val b = Any()

    @Test
    fun aFreshProcessStartsRos() {
        val guard = RosStackGuard()
        assertFalse(guard.ownedByOther(a))
        assertTrue(guard.claim(a) {})
    }

    @Test
    fun theOwnerMayClaimAgain() {
        val guard = RosStackGuard()
        assertTrue(guard.claim(a) {})
        assertTrue(guard.claim(a) {})
        assertFalse(guard.ownedByOther(a))
    }

    @Test
    fun aSecondInstanceIsRefused() {
        val guard = RosStackGuard()
        assertTrue(guard.claim(a) {})
        assertTrue(guard.ownedByOther(b))
        assertFalse(guard.claim(b) {})
        // The refusal does not take the stack over.
        assertFalse(guard.ownedByOther(a))
        assertTrue(guard.ownedByOther(b))
    }

    @Test
    fun theDoubleTapSequence() {
        // A is created and waits for the network; B is created in the same process before A's
        // network callback; A then starts ROS, and B's callback must not start a second stack.
        val guard = RosStackGuard()
        assertFalse(guard.ownedByOther(a)) // A.onCreate
        assertFalse(guard.ownedByOther(b)) // B.onCreate: nobody runs ROS yet
        assertTrue(guard.claim(a) {}) // A.startRos
        assertFalse(guard.claim(b) {}) // B.startRos: restart instead
    }

    @Test
    fun theFirstClaimWins() {
        // Either instance may reach startRos first; whichever does owns the stack.
        val guard = RosStackGuard()
        assertTrue(guard.claim(b) {})
        assertFalse(guard.claim(a) {})
    }

    @Test
    fun aRestartFromAnotherInstanceStopsTheOwner() {
        val guard = RosStackGuard()
        var aStops = 0
        var bStops = 0
        assertTrue(guard.claim(a) { aStops++ })
        assertFalse(guard.claim(b) { bStops++ })
        assertTrue(guard.stopOwnerFor(b))
        assertEquals(1, aStops)
        assertEquals(0, bStops) // a refused claim's stop is not kept
    }

    @Test
    fun theOwnerDoesNotStopItselfTwice() {
        // The owner's own restart already ran its stop; stopOwnerFor adds nothing.
        val guard = RosStackGuard()
        var aStops = 0
        assertTrue(guard.claim(a) { aStops++ })
        assertFalse(guard.stopOwnerFor(a))
        assertEquals(0, aStops)
    }

    @Test
    fun nothingToStopBeforeAnyClaim() {
        assertFalse(RosStackGuard().stopOwnerFor(a))
    }

    @Test
    fun theRestartedProcessStartsRosNormally() {
        // The guard is per process: after the restart, the new process's guard is empty, so the
        // relaunched instance starts ROS and does not restart again.
        val before = RosStackGuard()
        assertTrue(before.claim(a) {})
        assertFalse(before.claim(b) {})
        val after = RosStackGuard()
        val c = Any()
        assertFalse(after.ownedByOther(c))
        assertTrue(after.claim(c) {})
    }
}
