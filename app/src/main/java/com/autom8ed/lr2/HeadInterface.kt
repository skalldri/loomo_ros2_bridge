package com.autom8ed.lr2

import android.util.Log
import com.segway.robot.sdk.base.bind.ServiceBinder
import com.segway.robot.sdk.locomotion.head.Head

class HeadInterface constructor(ctx: android.content.Context) {
    private var mBindHeadListener: ServiceBinder.BindStateListener;
    private var mHead: Head = Head.getInstance()
    private val TAG: String = "HeadIface"
    init {
        mBindHeadListener = object : ServiceBinder.BindStateListener {
            override fun onBind() {
                Log.d(TAG, "mBindHeadListener onBind() called")
            }

            override fun onUnbind(reason: String) {
                Log.d(TAG, "mBindHeadListener onUnbind() called with: reason = [$reason]")
            }
        }
        mHead.bindService(ctx, mBindHeadListener)
    }
}