package com.autom8ed.lr2

import android.media.MediaPlayer
import android.util.Log
import java.util.concurrent.CountDownLatch
import java.util.concurrent.LinkedBlockingDeque

class AudioInterface(ctx: android.content.Context, node: RosNode) {
        var mNode = node
        var mPlayQueue = LinkedBlockingDeque<String>()
        val mCtx = ctx
        val TAG = "AudioInterface"

        // Spawn a thread to manage playback
        val mPlaybackThread: Thread = Thread {
            while (true) {
                // Take a string from the play queue
                val resource = mPlayQueue.take()

                val resourceId = mCtx.resources.getIdentifier(resource, "raw", mCtx.packageName)
                if (resourceId != 0) {
                    val mediaPlayer = MediaPlayer.create(mCtx, resourceId)
                    // Create a CountDownLatch initialized to 1
                    val latch = CountDownLatch(1)

                    mediaPlayer.setOnCompletionListener {
                        // When playback is complete, count down the latch to unblock the thread
                        latch.countDown()
                    }

                    mediaPlayer.start()

                    // Block: wait for playback to complete
                    latch.await()
                }
                else
                {
                    Log.e(TAG, "Failed to identify resource for '$resource'")
                }
            }
        }

    init {
        mPlaybackThread.start()

        mNode.node.createSubscription(std_msgs.msg.String::class.java, "/loomo/bark") { msg: std_msgs.msg.String ->
            mPlayQueue.addLast(msg.data)
        }
    }
}