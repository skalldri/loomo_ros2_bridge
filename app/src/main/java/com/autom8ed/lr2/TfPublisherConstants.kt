package com.autom8ed.lr2

object TfPublisherConstants {
    const val BASE_ODOM = "base_odom"
    const val BASE_LINK = "base_link"
    const val NECK_LINK = "neck_link"
    const val FISHEYE_FRAME_ID = "loomo_fisheye"
    const val FISHEYE_OPTICAL_FRAME_ID = "loomo_fisheye_optical"
    const val REALSENSE_DEPTH_FRAME_ID = "loomo_realsense_depth"
    const val REALSENSE_COLOR_FRAME_ID = "loomo_realsense_color"
    const val REALSENSE_DEPTH_OPTICAL_FRAME_ID = "loomo_realsense_depth_optical"
    const val REALSENSE_COLOR_OPTICAL_FRAME_ID = "loomo_realsense_color_optical"
    const val ULTRASONIC_FRAME_ID = "loomo_ultrasonic"
    const val IR_LEFT_FRAME_ID = "loomo_ir_left"
    const val IR_RIGHT_FRAME_ID = "loomo_ir_right"
    const val LOOMO_ODOM = "loomo_odom"

    const val ULTRASONIC_HEIGHT_M = 0.44
    const val ULTRASONIC_FORWARD_M = 0.1

    const val IR_CENTER_OFFSET_M = 0.025
    const val IR_HEIGHT_OFFSET_FROM_USS_M = -0.035
}