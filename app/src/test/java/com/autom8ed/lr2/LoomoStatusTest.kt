package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LoomoStatusTest {
    private fun value(s: LoomoStatus.Status, key: String) = s.values.toMap()[key]

    @Test
    fun batteryLevelsFollowTheThresholds() {
        assertEquals(LoomoStatus.OK, LoomoStatus.battery(100, false).level)
        assertEquals(LoomoStatus.OK, LoomoStatus.battery(20, false).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.battery(19, false).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.battery(10, true).level)
        assertEquals(LoomoStatus.ERROR, LoomoStatus.battery(9, true).level)
        assertEquals(LoomoStatus.ERROR, LoomoStatus.battery(0, null).level)
    }

    @Test
    fun batteryMessageAndValues() {
        val s = LoomoStatus.battery(15, true)
        assertEquals(LoomoStatus.BATTERY, s.name)
        assertEquals("15 %, charging (low)", s.message)
        assertEquals("15", value(s, "percent"))
        assertEquals("true", value(s, "charging"))
        assertEquals("80 %", LoomoStatus.battery(80, null).message)
        assertEquals("unknown", value(LoomoStatus.battery(80, null), "charging"))
    }

    @Test
    fun noOrInvalidBatteryReadingIsStale() {
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(null, null).level)
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(-1, false).level)
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(101, false).level)
    }

    @Test
    fun baseStateLevels() {
        val fresh = 1_000_000_000L
        assertEquals(LoomoStatus.OK, LoomoStatus.baseState(LoomoStatus.BaseState(mode = 0), fresh).level)
        assertEquals("normal", LoomoStatus.baseState(LoomoStatus.BaseState(), fresh).message)
        for (s in listOf(
            LoomoStatus.BaseState(lifted = true),
            LoomoStatus.BaseState(pushing = true),
            LoomoStatus.BaseState(locked = true),
        )) {
            assertEquals(s.toString(), LoomoStatus.WARN, LoomoStatus.baseState(s, fresh).level)
        }
        for (s in listOf(
            LoomoStatus.BaseState(fallen = true),
            LoomoStatus.BaseState(stuck = true, lifted = true),
        )) {
            assertEquals(s.toString(), LoomoStatus.ERROR, LoomoStatus.baseState(s, fresh).level)
        }
        assertEquals("stuck, lifted", LoomoStatus.baseState(LoomoStatus.BaseState(stuck = true, lifted = true), fresh).message)
        // Reported as values only.
        assertEquals(LoomoStatus.OK, LoomoStatus.baseState(LoomoStatus.BaseState(rider = true, steppedOn = true), fresh).level)
    }

    @Test
    fun baseStateGoesStale() {
        assertEquals(LoomoStatus.STALE, LoomoStatus.baseState(null, 0).level)
        val old = LoomoStatus.baseState(LoomoStatus.BaseState(fallen = true), LoomoStatus.BASE_STATE_STALE_NS + 1)
        assertEquals(LoomoStatus.STALE, old.level)
        assertEquals("true", value(old, "fallen"))
    }

    @Test
    fun modeNames() {
        assertEquals("robot", value(LoomoStatus.baseState(LoomoStatus.BaseState(mode = 0), 0), "mode"))
        assertEquals("self-balancing vehicle", LoomoStatus.modeName(1))
        assertEquals("unknown", LoomoStatus.modeName(-1))
    }

    private fun health(
        base: Boolean = true,
        sensor: Boolean = true,
        head: Boolean = true,
        synced: Boolean = true,
        link: JetsonLink.Status = JetsonLink.Status(JetsonLink.State.FULL, 1000),
    ) = LoomoStatus.AppHealth(base, sensor, head, synced, 1_234_567L, link, "1.0")

    @Test
    fun appHealth() {
        val ok = LoomoStatus.app(health())
        assertEquals(LoomoStatus.OK, ok.level)
        assertEquals("running", ok.message)
        assertEquals("1.235", value(ok, "clock_offset_ms"))
        assertEquals("full", value(ok, "jetson_link"))

        assertEquals(LoomoStatus.ERROR, LoomoStatus.app(health(base = false)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.app(health(sensor = false)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.app(health(head = false)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.app(health(synced = false)).level)
        val slow = LoomoStatus.app(health(link = JetsonLink.Status(JetsonLink.State.SLOW, 100)))
        assertEquals(LoomoStatus.WARN, slow.level)
        assertTrue(slow.message, slow.message.contains("100 Mb/s"))
        val noLink = LoomoStatus.app(health(base = false, link = JetsonLink.Status(JetsonLink.State.UNKNOWN, null)))
        assertEquals(LoomoStatus.ERROR, noLink.level)
        assertEquals("base unbound, no link report from the Jetson", noLink.message)
        assertEquals("unknown", value(noLink, "jetson_link_mbps"))
    }

    @Test
    fun everyNameHasThePrefix() {
        for (n in listOf(LoomoStatus.BATTERY, LoomoStatus.BASE_STATE, LoomoStatus.APP)) {
            assertTrue(n, n.startsWith("loomo: "))
        }
    }
}
