package com.autom8ed.lr2

import org.junit.Assert.assertEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class LoomoStatusTest {
    private fun value(s: LoomoStatus.Status, key: String) = s.values.toMap()[key]

    @Test
    fun batteryLevelsFollowTheThresholds() {
        assertEquals(LoomoStatus.OK, LoomoStatus.battery(100).level)
        assertEquals(LoomoStatus.OK, LoomoStatus.battery(20).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.battery(19).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.battery(10).level)
        assertEquals(LoomoStatus.ERROR, LoomoStatus.battery(9).level)
        assertEquals(LoomoStatus.ERROR, LoomoStatus.battery(0).level)
    }

    @Test
    fun batteryMessageAndValues() {
        val s = LoomoStatus.battery(15)
        assertEquals(LoomoStatus.BATTERY, s.name)
        assertEquals("15 % (low)", s.message)
        assertEquals("15", value(s, "percent"))
        assertEquals("80 %", LoomoStatus.battery(80).message)
    }

    @Test
    fun noOrInvalidBatteryReadingIsStale() {
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(null).level)
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(-1).level)
        assertEquals(LoomoStatus.STALE, LoomoStatus.battery(101).level)
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
        for (n in listOf(LoomoStatus.BATTERY, LoomoStatus.HEAD_BATTERY, LoomoStatus.BASE_STATE, LoomoStatus.APP)) {
            assertTrue(n, n.startsWith("loomo: "))
        }
    }

    // As read on the Loomo on 2026-11-24 (dumpsys battery): 95 %, 4.180 V, 20.0 C, good, full, AC.
    private fun head(
        percent: Int = 95, tempDeciC: Int = 200, health: Int = LoomoStatus.ANDROID_HEALTH_GOOD,
        status: Int = 5, plugged: Int = 1,
    ) = LoomoStatus.HeadBattery(percent, 4180, tempDeciC, health, status, plugged)

    @Test
    fun headBattery() {
        val ok = LoomoStatus.headBattery(head())
        assertEquals(LoomoStatus.HEAD_BATTERY, ok.name)
        assertEquals(LoomoStatus.OK, ok.level)
        assertEquals("95 %, full", ok.message)
        assertEquals("4.180", value(ok, "voltage_v"))
        assertEquals("20.0", value(ok, "temperature_c"))
        assertEquals("good", value(ok, "health"))
        assertEquals("true", value(ok, "external_power"))

        assertEquals(LoomoStatus.STALE, LoomoStatus.headBattery(null).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.headBattery(head(plugged = 0)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.headBattery(head(percent = 15)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.headBattery(head(tempDeciC = 450)).level)
        assertEquals(LoomoStatus.OK, LoomoStatus.headBattery(head(tempDeciC = 449)).level)
        assertEquals(LoomoStatus.WARN, LoomoStatus.headBattery(head(health = LoomoStatus.ANDROID_HEALTH_COLD)).level)
        assertEquals(LoomoStatus.ERROR, LoomoStatus.headBattery(head(percent = 5)).level)
        val hot = LoomoStatus.headBattery(head(health = LoomoStatus.ANDROID_HEALTH_OVERHEAT, tempDeciC = 600))
        assertEquals(LoomoStatus.ERROR, hot.level)
        assertEquals("health overheat, 60.0 °C", hot.message)
    }
}
