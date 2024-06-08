package com.autom8ed.loomoros2bridge

class Orchestrator {
    /*
    Thinking behind this class:
    - Whenever we get a new data payload (camera, sensor, etc...) we need to publish associated
    TF / calibration messages alongside it
     */

    /**
     *
     */
    public fun onCameraImage() {

        // - Query the TF module for the TF at time=image capture time
        // - Get the relevant calibration data for the camera (or should we only publish this once?)
        // - Publish the camera image + TF data

        // Topics:
        // - /camera/image_raw
        // - /camera/camera_info
        //
        // Loomo ROS Topic Structure
        //
        // /loomo
        //       /fisheye
        //               /camera_info
        //               /image_raw
        //       /realsense
        //                 /rgb
        //                     /camera_info
        //                     /image_raw
        //                 /depth
        //                       /camera_info
        //                       /image_raw
        //       /imu
        //           /data
        //

    }
}