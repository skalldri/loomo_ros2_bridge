package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.PerfCounter
import com.autom8ed.lr2.RosNode
import org.ros2.rcljava.qos.QoSProfile
import java.nio.ByteBuffer
import java.util.concurrent.Semaphore
import java.util.concurrent.TimeUnit

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
    private val mBitmap: Bitmap = Bitmap.createBitmap(
        mCamera.getResolution().mWidth,
        mCamera.getResolution().mHeight,
        mCamera.getImageType().getBitmapConfig()
    )
    private val mByteBuffer: ByteBuffer = ByteBuffer.allocate(mBitmap.byteCount)
    private var mMediaCodec: MediaCodec = MediaCodec.createByCodecName("OMX.Intel.hw_ve.h264")
    private var mMediaCodecReady: Boolean = false
    private val mMediaCodecSem: Semaphore = Semaphore(1)
    private val kYuv420Size: Int = ((mCamera.getResolution().mWidth * mCamera.getResolution().mHeight) * 3) / 2
    private val mPerfCounter: PerfCounter = PerfCounter("H264Publisher - $mTopic")
    private val rgbToYuvPerf: PerfCounter = PerfCounter("H264Publisher - $mTopic - RGB2YUV")

    override val TAG = "H264Publisher - $mTopic"

    init {
        for (t in mMediaCodec.codecInfo.supportedTypes) {
            Log.i(TAG, "Supported type: $t")
        }

        val caps = mMediaCodec.codecInfo.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC)
        Log.i(TAG, "Capabilities: " + caps.encoderCapabilities.toString())
    }

    private fun configureAndStartMediaCodec() {
        val mediaFormat: MediaFormat =
            MediaFormat.createVideoFormat(
                MediaFormat.MIMETYPE_VIDEO_AVC,
                mCamera.getResolution().mWidth,
                mCamera.getResolution().mHeight
            )
        mediaFormat.setInteger(MediaFormat.KEY_BIT_RATE, 200000000)
        mediaFormat.setInteger(MediaFormat.KEY_FRAME_RATE, 5)
        mediaFormat.setInteger(
            MediaFormat.KEY_COLOR_FORMAT,
            MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible
        )

        mediaFormat.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 1)
        // mediaFormat.setInteger(MediaFormat.KEY_PROFILE, MediaCodecInfo.CodecProfileLevel.AVCProfileBaseline)
        // mediaFormat.setInteger(MediaFormat.KEY_LEVEL, MediaCodecInfo.CodecProfileLevel.AVCLevel4)

        mMediaCodec.configure(mediaFormat, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        val outFmt = mMediaCodec.outputFormat
        val inFmt = mMediaCodec.inputFormat
        Log.i(TAG, "Configured media codec!")

        mMediaCodec.start()
    }

    private fun publishCompressedImage(byteBuffer: ByteBuffer, platformTimeStamp: Long) {
        // Assign to ROS msg
        val msg = sensor_msgs.msg.CompressedImage()

        // ByteBuffers returned from MediaCodec are read-only, so the internal array is not accessible.
        // Surely one more memcpy() won't kill us at this point....
        val plainByteArray = ByteArray(byteBuffer.remaining())
        byteBuffer.get(plainByteArray)
        msg.data = plainByteArray

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

        mPerfCounter.start()

        val hasSubscribers: Boolean = hasSubscribers()

        mMediaCodecSem.acquire()
        if (hasSubscribers && mMediaCodecReady) {
            // Reset position within the byte-buffer so that we overwrite everything
            mByteBuffer.clear()

            // Copy from Bitmap -> Buffer
            bitmap.copyPixelsToBuffer(mByteBuffer)

            val inputBufferIndex: Int = mMediaCodec.dequeueInputBuffer(0)

            if (inputBufferIndex < 0) {
                Log.e(TAG, "No input buffer available for MediaCodec!")
                mMediaCodecSem.release()
                return
            }

            // Assert non-NULL: since we got a valid index above, this should always return a valid buffer
            val inputBuffer: ByteBuffer = mMediaCodec.getInputBuffer(inputBufferIndex)!!

            rgbToYuvPerf.start()
            encodeYUV420SP(
                inputBuffer,
                mByteBuffer,
                mCamera.getResolution().mWidth,
                mCamera.getResolution().mHeight
            )
            rgbToYuvPerf.stop()

            mMediaCodec.queueInputBuffer(inputBufferIndex, 0, kYuv420Size, 0, 0)

            do {
                val bufferInfo: MediaCodec.BufferInfo = MediaCodec.BufferInfo()
                val outputBufferIndex: Int = mMediaCodec.dequeueOutputBuffer(bufferInfo, 50000)

                if (outputBufferIndex >= 0) {
                    val outputBuffer: ByteBuffer = mMediaCodec.getOutputBuffer(outputBufferIndex)!!
                    publishCompressedImage(outputBuffer, platformTimeStamp)
                    mMediaCodec.releaseOutputBuffer(outputBufferIndex, false)

                    if (bufferInfo.flags and MediaCodec.BUFFER_FLAG_CODEC_CONFIG == MediaCodec.BUFFER_FLAG_CODEC_CONFIG) {
                        // We need to continue sending compressed packets until we have drained all the CSV packets.
                        // Continue after sending this packet
                        // Log.i(TAG, "Got Codec-Specific Value buffer!")
                        continue
                    }

                    // Log.i(TAG, "Probably final buffer! Send complete")
                    break
                } else if (outputBufferIndex == MediaCodec.INFO_OUTPUT_FORMAT_CHANGED) {
                    // Log.i(TAG, "MediaCodec reports output format change!")
                    continue
                }
                else {
                    Log.e(TAG, "Did not get a valid output buffer from MediaCodec!")
                    break
                }
            } while(true)
        }
        mMediaCodecSem.release()

        mPerfCounter.stop()
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

    override fun onSubscriptionStateChange(hasSubscribers: Boolean) {
        mMediaCodecSem.acquire()

        if (hasSubscribers) {
            Log.i(TAG, "H.264 subscriber detected! Starting MediaCodec...")
            configureAndStartMediaCodec()
            mMediaCodecReady = true
        }
        else {
            Log.i(TAG, "H.264 detected loss of all subscribers! Stopping MediaCodec...")
            mMediaCodec.stop()
            mMediaCodecReady = false
        }

        mMediaCodecSem.release()
    }
}