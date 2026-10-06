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
 *
 * Encoded output is collected and published on a thread of its own. The Vision SDK hands the
 * next frame to the listener only once the callback returns, and waiting for the encoder's
 * output in the callback (~15-20 ms) was enough to miss every third frame of the 30 fps stream
 * (frame timestamps showed 33 ms and 67 ms gaps, nothing else). The callback now only draws.
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
    @Volatile private var mCodecConfig: ByteArray? = null
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

    // All MediaCodec calls go through this lock: the drain thread owns output buffers, the
    // camera thread only queues input buffers on the fallback path (Surface drawing needs no
    // codec call). Not mMediaCodecSem, which stopping the encoder holds while joining the thread.
    private val mCodecLock = Any()
    private var mDrainThread: Thread? = null
    @Volatile private var mDraining: Boolean = false

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
        synchronized(mPendingStamps) { mPendingStamps.clear() }
        mDraining = true
        mDrainThread = Thread({ drainLoop() }, "h264-drain").apply { start() }
        Log.i(TAG, "Encoder started (${if (mUseSurfaceInput) "Surface" else "buffer"} input, " +
            "${kBitRate / 1000} kbit/s, I-frame every ${kIFrameIntervalSeconds}s)")
    }

    private fun stopMediaCodec() {
        mDraining = false
        mDrainThread?.join(1000)
        mDrainThread = null
        synchronized(mCodecLock) {
            mMediaCodec.stop()
        }
        mInputSurface?.release()
        mInputSurface = null
        // Keep mCodecConfig: this encoder emits its codec-config buffer (SPS/PPS) only on the
        // first start after creation. The encoder is stopped and restarted every time the
        // subscriber count drops to zero and comes back (a DDS-Router restart does that), and
        // the stream format never changes, so the cached parameter sets stay valid. Clearing
        // them here left every keyframe after a restart without SPS/PPS and the Jetson decoder
        // waiting forever.
        synchronized(mPendingStamps) { mPendingStamps.clear() }
    }

    private fun drainLoop() {
        while (mDraining) {
            try {
                if (!drainOne(10000L)) {
                    continue
                }
            } catch (e: Exception) {
                if (mDraining) {
                    Log.e(TAG, "Encoder output drain failed", e)
                }
                return
            }
        }
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
                if (mUseSurfaceInput && surface != null) {
                    feedSurface(surface, bitmap, platformTimeStamp)
                } else {
                    feedBuffer(bitmap, platformTimeStamp)
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
                // Record the stamp before posting: the drain thread can dequeue the encoded
                // frame before this call returns, and must find the entry already there.
                rememberStamp(System.nanoTime() / 1000, platformTimeStamp)
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
        return true
    }

    // Fallback: RGBA -> NV12 on the CPU into an encoder input buffer.
    private fun feedBuffer(bitmap: Bitmap, platformTimeStamp: Long): Boolean {
        mByteBuffer.clear()
        bitmap.copyPixelsToBuffer(mByteBuffer)
        synchronized(mCodecLock) {
            val inputBufferIndex: Int = mMediaCodec.dequeueInputBuffer(0)
            if (inputBufferIndex < 0) {
                Log.e(TAG, "No input buffer available for MediaCodec!")
                return false
            }
            val inputBuffer: ByteBuffer = mMediaCodec.getInputBuffer(inputBufferIndex)!!
            mDrawPerf.start()
            encodeYUV420SP(inputBuffer, mByteBuffer, mWidth, mHeight)
            mDrawPerf.stop()
            rememberStamp(platformTimeStamp, platformTimeStamp)
            mMediaCodec.queueInputBuffer(inputBufferIndex, 0, kYuv420Size, platformTimeStamp, 0)
        }
        return true
    }

    private fun rememberStamp(presentationTimeUs: Long, platformTimeStamp: Long) {
        synchronized(mPendingStamps) {
            mPendingStamps.addLast(Pair(presentationTimeUs, platformTimeStamp))
            while (mPendingStamps.size > kMaxPendingStamps) {
                mPendingStamps.removeFirst()
            }
        }
    }

    // Platform timestamp of the input frame closest to this output's presentation time; entries
    // up to and including the match are dropped (output is in presentation order). Falls back
    // to the current platform clock if nothing is pending.
    private fun stampFor(presentationTimeUs: Long): Long {
        synchronized(mPendingStamps) {
            var best: Pair<Long, Long>? = null
            for (entry in mPendingStamps) {
                if (best == null || Math.abs(entry.first - presentationTimeUs) < Math.abs(best.first - presentationTimeUs)) {
                    best = entry
                }
            }
            if (best == null) {
                return System.nanoTime() / 1000
            }
            while (mPendingStamps.isNotEmpty()) {
                val head = mPendingStamps.removeFirst()
                if (head === best) {
                    break
                }
            }
            return best.second
        }
    }

    // Take one encoded buffer from the encoder (waiting up to timeoutUs) and publish it.
    // Returns false if nothing was ready. Runs on the drain thread.
    private fun drainOne(timeoutUs: Long): Boolean {
        val bufferInfo = MediaCodec.BufferInfo()
        val frame: ByteArray
        synchronized(mCodecLock) {
            val outputBufferIndex: Int = mMediaCodec.dequeueOutputBuffer(bufferInfo, timeoutUs)
            if (outputBufferIndex < 0) {
                // INFO_TRY_AGAIN_LATER, INFO_OUTPUT_FORMAT_CHANGED, INFO_OUTPUT_BUFFERS_CHANGED
                return false
            }
            val outputBuffer: ByteBuffer = mMediaCodec.getOutputBuffer(outputBufferIndex)!!
            // ByteBuffers returned from MediaCodec are read-only, so the internal array is
            // not accessible. Surely one more memcpy() won't kill us at this point....
            frame = ByteArray(bufferInfo.size)
            outputBuffer.position(bufferInfo.offset)
            outputBuffer.limit(bufferInfo.offset + bufferInfo.size)
            outputBuffer.get(frame)
            mMediaCodec.releaseOutputBuffer(outputBufferIndex, false)
        }

        if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG != 0) {
            // Codec-specific data (SPS/PPS) as its own buffer: keep it for the keyframes.
            Log.i(TAG, "Got codec config (SPS/PPS), ${frame.size} bytes")
            mCodecConfig = frame
            return true
        }

        // With Surface input this encoder emits no separate codec-config buffer and carries
        // SPS/PPS inline in the first IDR frame only, so also harvest them from the frame
        // itself, and look for an IDR slice anywhere in it, not just first.
        val nals = scanNalUnits(frame)
        val inlineConfig = extractParameterSets(frame, nals)
        if (inlineConfig != null) {
            if (mCodecConfig == null) {
                Log.i(TAG, "Got inline SPS/PPS, ${inlineConfig.size} bytes")
            }
            mCodecConfig = inlineConfig
        }

        val stamp = stampFor(bufferInfo.presentationTimeUs)
        val config = mCodecConfig
        val isKeyFrame = (bufferInfo.flags and MediaCodec.BUFFER_FLAG_KEY_FRAME != 0) ||
            nals.any { it.second == NAL_IDR }
        if (isKeyFrame && config != null && inlineConfig == null) {
            publishCompressedImage(config + frame, stamp)
        } else {
            publishCompressedImage(frame, stamp)
        }
        return true
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

    // (start offset of the start code, NAL type) for every NAL unit in an Annex B buffer.
    private fun scanNalUnits(data: ByteArray): List<Pair<Int, Int>> {
        val nals = ArrayList<Pair<Int, Int>>()
        var i = 0
        while (i < data.size - 3) {
            if (data[i].toInt() == 0 && data[i + 1].toInt() == 0 && data[i + 2].toInt() == 1) {
                // Prefer the 4-byte start code (00 00 00 01) as the unit boundary when present.
                val start = if (i > 0 && data[i - 1].toInt() == 0) i - 1 else i
                nals.add(Pair(start, data[i + 3].toInt() and 0x1f))
                i += 3
            } else {
                i++
            }
        }
        return nals
    }

    // The SPS and PPS NAL units of a frame, as one Annex B blob, or null if it has none.
    private fun extractParameterSets(data: ByteArray, nals: List<Pair<Int, Int>>): ByteArray? {
        var out = ByteArray(0)
        for ((index, nal) in nals.withIndex()) {
            if (nal.second == NAL_SPS || nal.second == NAL_PPS) {
                val end = if (index + 1 < nals.size) nals[index + 1].first else data.size
                out += data.copyOfRange(nal.first, end)
            }
        }
        return if (out.isEmpty()) null else out
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
            mMediaCodecReady = false
            stopMediaCodec()
        }

        mMediaCodecSem.release()
    }

    companion object {
        // 640x480 at the camera's frame rate; the previous 200 Mbit/s setting was meaningless to
        // the rate control (it produced ~85 KB P-frames regardless).
        private const val kBitRate = 4000000
        private const val kIFrameIntervalSeconds = 1
        private const val kMaxPendingStamps = 16
        private const val NAL_IDR = 5
        private const val NAL_SPS = 7
        private const val NAL_PPS = 8
    }
}
