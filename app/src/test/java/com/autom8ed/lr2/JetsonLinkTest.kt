package com.autom8ed.lr2

import com.autom8ed.lr2.JetsonLink.State
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test

private const val MS = 1_000_000L
private const val T0 = 1_000L * MS

class JetsonLinkTest {

    private val link = JetsonLink()

    @Test
    fun unknownBeforeTheFirstReport() {
        assertEquals(JetsonLink.Status(State.UNKNOWN, null), link.status(T0))
        assertNull(link.warning(T0))
    }

    @Test
    fun noWarningAtFullSpeed() {
        link.onReport(1000, T0)
        assertEquals(JetsonLink.Status(State.FULL, 1000), link.status(T0))
        assertNull(link.warning(T0))
    }

    @Test
    fun noWarningAboveFullSpeed() {
        link.onReport(2500, T0)
        assertEquals(State.FULL, link.status(T0).state)
        assertNull(link.warning(T0))
    }

    @Test
    fun warnsAt100() {
        link.onReport(100, T0)
        assertEquals(JetsonLink.Status(State.SLOW, 100), link.status(T0))
        assertEquals(
            "Ethernet to the Jetson at 100 Mb/s: camera frames will drop. Reseat the cable at both ends.",
            link.warning(T0)
        )
    }

    @Test
    fun warnsJustBelowFullSpeedAnd10() {
        link.onReport(999, T0)
        assertEquals(State.SLOW, link.status(T0).state)
        link.onReport(10, T0)
        assertEquals(JetsonLink.warningText(10), link.warning(T0))
    }

    @Test
    fun noCarrierIsUnknownNotAWarning() {
        link.onReport(0, T0)
        assertEquals(JetsonLink.Status(State.UNKNOWN, 0), link.status(T0))
        assertNull(link.warning(T0))
    }

    @Test
    fun warningClearsWhenTheSpeedReturns() {
        link.onReport(100, T0)
        assertTrue(link.warning(T0) != null)
        link.onReport(1000, T0 + 1000 * MS)
        assertNull(link.warning(T0 + 1000 * MS))
    }

    @Test
    fun slowReportStillWarnsAt5s() {
        link.onReport(100, T0)
        assertEquals(JetsonLink.warningText(100), link.warning(T0 + 5000 * MS))
    }

    @Test
    fun silenceOfMoreThan5sHidesTheWarning() {
        link.onReport(100, T0)
        val later = T0 + 5000 * MS + 1
        assertEquals(JetsonLink.Status(State.UNKNOWN, null), link.status(later))
        assertNull(link.warning(later))
    }

    @Test
    fun aFreshReportAfterSilenceWarnsAgain() {
        link.onReport(100, T0)
        assertNull(link.warning(T0 + 10_000 * MS))
        link.onReport(100, T0 + 10_000 * MS)
        assertEquals(JetsonLink.warningText(100), link.warning(T0 + 10_500 * MS))
    }

    @Test
    fun repeatsKeepTheWarningUp() {
        for (i in 0..30) {
            link.onReport(100, T0 + i * 1000 * MS)
            assertEquals(State.SLOW, link.status(T0 + i * 1000 * MS + 999 * MS).state)
        }
    }

    @Test
    fun fullSpeedGoesUnknownAfterSilenceToo() {
        link.onReport(1000, T0)
        assertEquals(State.FULL, link.status(T0 + 5000 * MS).state)
        assertEquals(State.UNKNOWN, link.status(T0 + 5001 * MS).state)
    }

    @Test
    fun onReportSaysWhenTheSpeedChanged() {
        assertTrue(link.onReport(1000, T0))
        assertFalse(link.onReport(1000, T0 + 1000 * MS))
        assertTrue(link.onReport(100, T0 + 2000 * MS))
        assertFalse(link.onReport(100, T0 + 3000 * MS))
        assertTrue(link.onReport(0, T0 + 4000 * MS))
    }

    @Test
    fun customThresholdAndStaleness() {
        val l = JetsonLink(staleNs = 2000 * MS, fullMbps = 100)
        l.onReport(100, T0)
        assertNull(l.warning(T0))
        l.onReport(10, T0)
        assertEquals(JetsonLink.warningText(10), l.warning(T0 + 2000 * MS))
        assertNull(l.warning(T0 + 2001 * MS))
    }

    @Test
    fun uint16IsReadUnsigned() {
        assertEquals(1000, JetsonLink.mbpsOf(1000.toShort()))
        assertEquals(0, JetsonLink.mbpsOf(0.toShort()))
        assertEquals(40000, JetsonLink.mbpsOf(40000.toShort()))
        assertEquals(65535, JetsonLink.mbpsOf((-1).toShort()))
    }

    @Test
    fun statsLine() {
        assertEquals("jetson_link state=UNKNOWN no report", link.statsLine(T0))
        link.onReport(100, T0)
        assertEquals("jetson_link state=SLOW last=100 Mb/s age=250 ms", link.statsLine(T0 + 250 * MS))
        assertEquals("jetson_link state=UNKNOWN last=100 Mb/s age=6000 ms", link.statsLine(T0 + 6000 * MS))
    }
}
