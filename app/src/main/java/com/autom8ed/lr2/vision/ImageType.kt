package com.autom8ed.lr2.vision

import android.graphics.Bitmap

const val IMAGE_ENCODING_MONO = "mono8"
const val IMAGE_ENCODING_DEPTH = "16UC1"
const val IMAGE_ENCODING_COLOR = "rgba8"

enum class ImageType {
    MONO {
        override fun supportsCompressedPublisher(): Boolean {
            return false
        }
        override fun supportsH264Publisher(): Boolean {
            return false
        }
        override fun supportsCompressedDepthPublisher(): Boolean {
            return false
        }

        override fun getBitmapConfig(): Bitmap.Config {
            return Bitmap.Config.ALPHA_8
        }

        override fun getRosEncoding(): String {
            return IMAGE_ENCODING_MONO
        }
    },
    COLOR {
        override fun supportsCompressedPublisher(): Boolean {
            return false
        }
        override fun supportsH264Publisher(): Boolean {
            return true
        }
        override fun supportsCompressedDepthPublisher(): Boolean {
            return false
        }

        override fun getBitmapConfig(): Bitmap.Config {
            return Bitmap.Config.ARGB_8888
        }

        override fun getRosEncoding(): String {
            return IMAGE_ENCODING_COLOR
        }
    },
    DEPTH {
        override fun supportsCompressedPublisher(): Boolean {
            return false
        }
        override fun supportsH264Publisher(): Boolean {
            return false
        }
        override fun supportsCompressedDepthPublisher(): Boolean {
            return true
        }

        override fun getBitmapConfig(): Bitmap.Config {
            return Bitmap.Config.RGB_565
        }

        override fun getRosEncoding(): String {
            return IMAGE_ENCODING_DEPTH
        }
    };

    abstract fun supportsCompressedPublisher(): Boolean
    abstract fun supportsH264Publisher(): Boolean
    abstract fun supportsCompressedDepthPublisher(): Boolean

    abstract fun getBitmapConfig(): Bitmap.Config

    abstract fun getRosEncoding(): String
}