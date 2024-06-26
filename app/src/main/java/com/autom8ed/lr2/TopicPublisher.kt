package com.autom8ed.lr2

import android.util.Log

class TopicPublisher<T : Any> constructor(topic: String) {

    val mTopic: String = topic;
    val TAG: String = "TopicPublisher"

    public fun publish(message: T) {
        // DO NOTHING, fake publisher
        Log.i(TAG, message.toString())
    }
}