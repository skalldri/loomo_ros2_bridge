package com.autom8ed.lr2.vision

import android.graphics.Bitmap
import android.media.MediaCodec
import android.media.MediaCodecInfo
import android.media.MediaFormat
import android.util.Log
import com.autom8ed.lr2.AdvancedPublisher
import com.autom8ed.lr2.RosNode
import com.autom8ed.lr2.TfPublisherConstants
import com.autom8ed.lr2.TimeSync
import com.segway.robot.sdk.vision.frame.Frame
import org.ros2.rcljava.qos.QoSProfile
import java.nio.ByteBuffer
import java.util.concurrent.TimeUnit

class H264ImagePublisher(
    node: RosNode,
    topic: String,
    camera: LoomoCamera,
    qos: QoSProfile = QoSProfile.SENSOR_DATA
) : AdvancedPublisher<sensor_msgs.msg.Image>(
    sensor_msgs.msg.Image::class.java,
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

    override val TAG = "H264Publisher - $mTopic"

    init {
        for (t in mMediaCodec.codecInfo.supportedTypes) {
            Log.i(TAG, "Supported type: $t")
        }

        val caps = mMediaCodec.codecInfo.getCapabilitiesForType(MediaFormat.MIMETYPE_VIDEO_AVC)
        Log.i(TAG, "Capabilities: " + caps.encoderCapabilities.toString())

        val mediaFormat: MediaFormat =
            MediaFormat.createVideoFormat(
                MediaFormat.MIMETYPE_VIDEO_AVC,
                mCamera.getResolution().mWidth,
                mCamera.getResolution().mHeight
            )
        mediaFormat.setInteger(MediaFormat.KEY_BIT_RATE, 125000);
        mediaFormat.setInteger(MediaFormat.KEY_FRAME_RATE, 30);
        mediaFormat.setInteger(
            MediaFormat.KEY_COLOR_FORMAT,
            MediaCodecInfo.CodecCapabilities.COLOR_FormatYUV420Flexible
        );
        mediaFormat.setInteger(MediaFormat.KEY_I_FRAME_INTERVAL, 5);
        mMediaCodec.configure(mediaFormat, null, null, MediaCodec.CONFIGURE_FLAG_ENCODE)
        val outFmt = mMediaCodec.outputFormat
        val inFmt = mMediaCodec.inputFormat
        Log.i(TAG, "Configured media codec!")
    }

    fun publish(bitmap: Bitmap, platformTimeStamp: Long) {
        if (hasSubscribers()) {
            Log.i(TAG, "I want to publish H.264 image now!")
        }
    }
}