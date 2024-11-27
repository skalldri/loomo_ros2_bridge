package com.autom8ed.lr2

import org.junit.Test

import org.junit.Assert.*

import com.autom8ed.lr2.vision.getCameraInfoTopic
import com.autom8ed.lr2.vision.getCompressedDepthTopic
import com.autom8ed.lr2.vision.getCompressedTopic
import com.autom8ed.lr2.vision.getH264Topic

class ImageTransportUnitTests {
    @Test
    fun test_getCameraInfoTopic() {
        assertEquals("/a/b/camera_info", getCameraInfoTopic("/a/b/image"))
    }

    @Test
    fun test_getCompressedTopic() {
        assertEquals("/a/b/image/compressed", getCompressedTopic("/a/b/image"))
    }

    @Test
    fun test_getH264Topic() {
        assertEquals("/a/b/image/h264", getH264Topic("/a/b/image"))
    }

    @Test
    fun test_getCompressedDepthTopic() {
        assertEquals("/a/b/image/compressedDepth", getCompressedDepthTopic("/a/b/image"))
    }
}