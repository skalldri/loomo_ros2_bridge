package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import android.view.Surface
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile
import java.nio.ByteBuffer
import java.util.ArrayDeque
import java.util.concurrent.Semaphore
import java.util.concurrent.TimeUnit

/**
 * Publishes the colour stream as H.264 (sensor_msgs/CompressedImage, format "h264", Annex B).
 *
 * Frames go into the hardware encoder through its input Surface: the bitmap is drawn onto a
 * Canvas locked on that Surface and the encoder does the RGBA to YUV conversion. The previous
 * path converted RGBA to NV12 in a Kotlin per-pixel loop, which took ~130 ms per 640x480 frame on
 * the Loomo's Atom and capped every topic of the colour ImageTransport at ~3 Hz (the frame
 * listener thread is shared). That path is kept only as a fallback for an encoder that refuses
 * Surface input or lockCanvas(), which the MediaCodec documentation allows for.
 */
class H264ImagePublisher(
    node: RosNode,
    topic: String,
    camera: LoomoCamera,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) : AdvancedPublisher<sensor_msgs.msg.CompressedImage>(
    sensor_msgs.msg.CompressedImage::class.java,
    node,
    topic,
    qos
) {
    private val mCamera: LoomoCamera = camera
    private val mWidth: Int = mCamera.getResolution().mWidth
    private val mHeight: Int = mCamera.getResolution().mHeight
    private var mMediaCodec: MediaCodec = MediaCodec.createByCodecName("OMX.Intel.hw_ve.h264")
    private var mMediaCodecReady: Boolean = false
    // SPS/PPS from MediaCodec's BUFFER_FLAG_CODEC_CONFIG buffer. It is emitted once per encoder
    // start, so a decoder that subscribes later never sees it unless it is resent: the Isaac ROS
    // H.264 decoder on the Jetson waits forever for the stream parameters. Prepend it to every
    // keyframe instead of publishing it on its own.
    private var mCodecConfig: ByteArray? = null
    private val mMediaCodecSem: Semaphore = Semaphore(1)

    // Surface input (normal path). mUseSurfaceInput flips to false for good if the encoder
    // rejects it; the buffer path below then takes over on the next encoder start.
    private var mInputSurface: Surface? = null
    private var mUseSurfaceInput: Boolean = true

    // Buffer-input fallback path only.
    private val mByteBuffer: ByteBuffer = ByteBuffer.allocate(mWidth * mHeight * 4)
    private val kYuv420Size: Int = (mWidth * mHeight * 3) / 2

    // Encoder presentation time (us) -> platform timestamp of the frame that produced it, so the
    // published header carries the camera's timestamp. With Surface input the presentation time
    // is the system clock at unlockCanvasAndPost(), hence nearest-match rather than equality.
    private val mPendingStamps = ArrayDeque<Pair<Long, Long>>()

    private val mPerfCounter: PerfCounter = PerfCounter("H264Publisher - $mTopic")
    private val mDrawPerf: PerfCounter = PerfCounter("H264Publisher - $mTopic - draw")

    override val TAG = "H264Publisher - $mTopic"

    init {
        for (t in mMediaCodec.codecInfo.supportedTypes) {
            Log.i(TAG, "Supported type: $t")
        }

        val caps = mMediaCodec.codecInfo.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC)
        Log.i(TAG, "Capabilities: " + caps.encoderCapabilities.toString())
        Log.i(TAG, "Colour formats: " + caps.colorFormats.joinToString())
        Log.i(TAG, "Stream ${mWidth}x${mHeight} @ ${mCamera.getFps()} fps")

        // All fields used by onSubscriptionStateChange() exist from here on.
        enableSubscriptionStateCallbacks()
    }

    private fun configureAndStartMediaCodec() {
        val mediaFormat: MediaFormat =
            MediaFormat.createVideoFormat(MediaFormat.MIMETYPE_VIDEO_AVC, mWidth, mHeight)
        mediaFormat.setInteger(MediaFormat.KEY_BIT_RATE, kBitRate)
        // Frames are encoded as they arrive from the camera; this only steers rate control.
        mediaFormat.setInteger(MediaFormat.KEY_FRAME_RATE, mCamera.getFps())
        mediaFormat.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, kIFrameIntervalSeconds)
        mediaFormat.setInteger(
            MediaFormat.KEY_COLOR_FORMAT,
            if (mUseSurfaceInput) MediaCodecInfo.CodecCapabilities.COLOR_FormatSurface
            else MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible
        )

        mMediaCodec.configure(mediaFormat, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)

        if (mUseSurfaceInput) {
            try {
                mInputSurface = mMediaCodec.createInputSurface()
            } catch (e: Exception) {
                Log.e(TAG, "Encoder refused Surface input; falling back to buffer input", e)
                mUseSurfaceInput = false
                mMediaCodec.reset()
                configureAndStartMediaCodec()
                return
            }
        }

        mMediaCodec.start()
        mPendingStamps.clear()
        Log.i(TAG, "Encoder started (${if (mUseSurfaceInput) "Surface" else "buffer"} input, " +
            "${kBitRate / 1000} kbit/s, I-frame every ${kIFrameIntervalSeconds}s)")
    }

    private fun stopMediaCodec() {
        mMediaCodec.stop()
        mInputSurface?.release()
        mInputSurface = null
        mCodecConfig = null
        mPendingStamps.clear()
    }

    private fun publishCompressedImage(data: ByteArray, platformTimeStamp: Long) {
        // Assign to ROS msg
        val msg = sensor_msgs.msg.CompressedImage()
        msg.data = data

        msg.header.frameId = mCamera.getTfOpticalFrameId()
        msg.format = "h264"

        msg.header.stamp.sec =
            TimeUnit.SECONDS.convert(platformTimeStamp, TimeUnit.MICROSECONDS)
                .toInt()
        msg.header.stamp.nanosec =
            (platformTimeStamp % (1000 * 1000)).toInt() * (1000)

        publish(msg)
    }

    fun publish(bitmap: Bitmap, platformTimeStamp: Long) {
        if (!hasSubscribers()) {
            return
        }

        mPerfCounter.start()
        mMediaCodecSem.acquire()
        try {
            if (mMediaCodecReady) {
                val surface = mInputSurface
                val fed = if (mUseSurfaceInput && surface != null) {
                    feedSurface(surface, bitmap, platformTimeStamp)
                } else {
                    feedBuffer(bitmap, platformTimeStamp)
                }
                if (fed) {
                    drainOutput(platformTimeStamp)
                }
            }
        } finally {
            mMediaCodecSem.release()
        }
        mPerfCounter.stop()
    }

    // Draw the frame onto the encoder's input Surface. Returns false (and switches to buffer
    // input for the rest of the run) if the Surface cannot be locked for CPU drawing.
    private fun feedSurface(surface: Surface, bitmap: Bitmap, platformTimeStamp: Long): Boolean {
        mDrawPerf.start()
        try {
            val canvas = surface.lockCanvas(null)
            try {
                canvas.drawBitmap(bitmap, 0f, 0f, null)
            } finally {
                surface.unlockCanvasAndPost(canvas)
            }
        } catch (e: Exception) {
            Log.e(TAG, "lockCanvas() on the encoder input Surface failed; falling back to buffer input", e)
            mUseSurfaceInput = false
            stopMediaCodec()
            configureAndStartMediaCodec()
            return false
        } finally {
            mDrawPerf.stop()
        }
        rememberStamp(System.nanoTime() / 1000, platformTimeStamp)
        return true
    }

    // Fallback: RGBA -> NV12 on the CPU into an encoder input buffer.
    private fun feedBuffer(bitmap: Bitmap, platformTimeStamp: Long): Boolean {
        val inputBufferIndex: Int = mMediaCodec.dequeueInputBuffer(0)
        if (inputBufferIndex < 0) {
            Log.e(TAG, "No input buffer available for MediaCodec!")
            return false
        }
        val inputBuffer: ByteBuffer = mMediaCodec.getInputBuffer(inputBufferIndex)!!

        mByteBuffer.clear()
        bitmap.copyPixelsToBuffer(mByteBuffer)
        mDrawPerf.start()
        encodeYUV420SP(inputBuffer, mByteBuffer, mWidth, mHeight)
        mDrawPerf.stop()

        mMediaCodec.queueInputBuffer(inputBufferIndex, 0, kYuv420Size, platformTimeStamp, 0)
        rememberStamp(platformTimeStamp, platformTimeStamp)
        return true
    }

    private fun rememberStamp(presentationTimeUs: Long, platformTimeStamp: Long) {
        mPendingStamps.addLast(Pair(presentationTimeUs, platformTimeStamp))
        while (mPendingStamps.size > kMaxPendingStamps) {
            mPendingStamps.removeFirst()
        }
    }

    // Platform timestamp of the input frame closest to this output's presentation time; entries
    // up to and including the match are dropped (output is in presentation order).
    private fun stampFor(presentationTimeUs: Long, fallback: Long): Long {
        var best: Pair<Long, Long>? = null
        for (entry in mPendingStamps) {
            if (best == null || Math.abs(entry.first - presentationTimeUs) < Math.abs(best.first - presentationTimeUs)) {
                best = entry
            }
        }
        if (best == null) {
            return fallback
        }
        while (mPendingStamps.isNotEmpty()) {
            val head = mPendingStamps.removeFirst()
            if (head === best) {
                break
            }
        }
        return best.second
    }

    // Publish every encoded frame that is ready. Waits briefly for the first one (the encoder
    // usually returns the frame just fed), then takes whatever else is queued without waiting.
    private fun drainOutput(fallbackStamp: Long) {
        var timeoutUs = 50000L
        while (true) {
            val bufferInfo = MediaCodec.BufferInfo()
            val outputBufferIndex: Int = mMediaCodec.dequeueOutputBuffer(bufferInfo, timeoutUs)
            if (outputBufferIndex >= 0) {
                val outputBuffer: ByteBuffer = mMediaCodec.getOutputBuffer(outputBufferIndex)!!
                // ByteBuffers returned from MediaCodec are read-only, so the internal array is
                // not accessible. Surely one more memcpy() won't kill us at this point....
                val frame = ByteArray(bufferInfo.size)
                outputBuffer.position(bufferInfo.offset)
                outputBuffer.limit(bufferInfo.offset + bufferInfo.size)
                outputBuffer.get(frame)
                mMediaCodec.releaseOutputBuffer(outputBufferIndex, false)

                if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0) {
                    // Codec-specific data (SPS/PPS): keep it for the keyframes; the first real
                    // frame follows, so keep waiting for it.
                    Log.i(TAG, "Got codec config (SPS/PPS), ${frame.size} bytes")
                    mCodecConfig = frame
                    continue
                }

                val stamp = stampFor(bufferInfo.presentationTimeUs, fallbackStamp)
                val config = mCodecConfig
                val isKeyFrame = (bufferInfo.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0) || isIdrFrame(frame)
                if (isKeyFrame && config != null) {
                    publishCompressedImage(config + frame, stamp)
                } else {
                    publishCompressedImage(frame, stamp)
                }
                timeoutUs = 0
            } else if (outputBufferIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED ||
                outputBufferIndex == MediaCodec.INFO_OUTPUT_BUFFERS_CHANGED) {
                continue
            } else {
                // INFO_TRY_AGAIN_LATER: nothing (more) ready; the encoder may hold a frame in
                // flight, it comes out on the next call.
                break
            }
        }
    }

    private fun encodeYUV420SP(yuv420sp: ByteBuffer, rgba888: ByteBuffer, width: Int, height: Int) {
        val frameSize = width * height

        var yIndex = 0
        var uvIndex = frameSize

        var R: Int
        var G: Int
        var B: Int

        var Y: Int
        var U: Int
        var V: Int

        var index = 0
        for (row in 0 until height) {
            for (col in 0 until width) {
                R = rgba888[(row * width * 4) + (col * 4) + 0].toInt()
                G = rgba888[(row * width * 4) + (col * 4) + 1].toInt()
                B = rgba888[(row * width * 4) + (col * 4) + 2].toInt()
                // We don't care about the alpha channel
                // a = rgba[(row * width * 4) + (col * 4) + 3]

                // well known RGB to YUV algorithm
                Y = (0.257f*R + 0.504f*G + 0.098f*B + 16.0f).toInt()
                // Y = ((66 * R + 129 * G + 25 * B + 128) shr 8) + 16
                U = (-0.148f*R - 0.291f*G + 0.439f*B + 128.0f).toInt()
                // U = ((-38 * R - 74 * G + 112 * B + 128) shr 8) + 128
                V = (0.439f*R - 0.368f*G - 0.071f*B + 128.0f).toInt()
                // V = ((112 * R - 94 * G - 18 * B + 128) shr 8) + 128

                // NV21 has a plane of Y and interleaved planes of VU each sampled by a factor of 2
                //    meaning for every 4 Y pixels there are 1 V and 1 U.  Note the sampling is every other
                //    pixel AND every other scanline.
                if (Y > 255)
                {
                    Y = 255
                }

                if (U > 255)
                {
                    U = 255
                } else if (U < 0) {
                    U = 0
                }

                if (V > 255)
                {
                    V = 255
                } else if (V < 0) {
                    V = 0
                }

                yuv420sp.put(yIndex++, (Y).toByte())
                if (row % 2 == 0 && index % 2 == 0) {
                    yuv420sp.put(uvIndex++, (U).toByte())
                    yuv420sp.put(uvIndex++, (V).toByte())
                }

                index++
            }
        }
    }

    // True if the first NAL unit in an Annex B buffer is an IDR slice (type 5), for encoders that
    // do not set BUFFER_FLAG_KEY_FRAME.
    private fun isIdrFrame(data: ByteArray): Boolean {
        for (i in 0 until data.size - 3) {
            if (data[i].toInt() == 0 && data[i + 1].toInt() == 0 && data[i + 2].toInt() == 1) {
                return (data[i + 3].toInt() and 0x1f) == 5
            }
        }
        return false
    }

    override fun onSubscriptionStateChange(hasSubscribers: Boolean) {
        mMediaCodecSem.acquire()

        if (hasSubscribers) {
            Log.i(TAG, "H.264 subscriber detected! Starting MediaCodec...")
            configureAndStartMediaCodec()
            mMediaCodecReady = true
        }
        else {
            Log.i(TAG, "H.264 detected loss of all subscribers! Stopping MediaCodec...")
            stopMediaCodec()
            mMediaCodecReady = false
        }

        mMediaCodecSem.release()
    }

    companion object {
        // 640x480 at the camera's frame rate; the previous 200 Mbit/s setting was meaningless to
        // the rate control (it produced ~85 KB P-frames regardless).
        private const val kBitRate = 4000000
        private const val kIFrameIntervalSeconds = 1
        private const val kMaxPendingStamps = 16
    }
}
