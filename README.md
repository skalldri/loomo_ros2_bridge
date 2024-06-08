# loomo_ros2_bridge
This repository contains my latest attempt at a ROS2 <-> Segway Loomo bridge Android application.

## Why?
The Loomo is a pretty nice piece of hardware, but the onboard processing capabilities leave a lot to be desired. Offloading sensor processing tasks to another computer makes Loomo a lot more capable.

ROS2 is a logical choice to put on Loomo. It has a robust ecosystem of robotics algorithms available, and is still being updated.

## How?
Getting ROS2 on Loomo is a bit of a challenge. Loomo runs x86_64 Android 5.1, which is long past EOL. However, Android Studio can still produce binaries that run on this target.

ROS2 packages are another story. The [ROS2 Java / Android](https://github.com/ros2-java/ros2_java) project targets Java 1.8, while Android 5.1 only supports Java 1.6. I have a [fork](https://github.com/skalldri/ros2_java) that runs on Android 5.1.

## Build Process
> TODO: these instructions are a work-in-progress. 

The build process is a bit janky still. But the basic workflow is:
- Install ROS2 Humble. Tested on Ubuntu 22.04, Windows Subsystem for Linux
- Install the Android SDK 22 and latest Android NDK
- Clone my [fork of the ament_gradle plugin](https://github.com/skalldri/ament_gradle_plugin), which is required to build my fork of the `rclandroid` project
- Setup [my fork of the ROS2 Java project](https://github.com/skalldri/ros2_java) using the `ros2_java_android.repos` file
- `colcon build` the workspace
- Find the `rclandroid.aar` file that gets produced by the build and copy it into `deps/rclandroid.aar`

Now you should be able to just build the project using Android Studio. A copy of the `rclandroid.aar` file is included so you can skip this step, but it will likely only work on x86_64 Android targets.

## Loomo
The [Segway Ninebot Loomo](https://www.segway.com/loomo/) is a personal robot / personal transporter produced by Segway Ninebot.

Here are some relevant specs:
- Intel RealSense Color + Depth camera
- Fisheye camera (Seems to be black and white)
- 2x Infrared Depth sensors
- 1x Ultrasonic Depth sensor
- Robotic "head" with integrated Android tablet
- Speaker

## Preservation
The Loomo was discontinued at some point between 2019 and 2024. For now, the SDK documentation is still available [here](https://developer.segwayrobotics.com/developer/documents/segway-robots-sdk.html), and the SDK libraries can still be downloaded from the online Maven repositories.

I'm partially working on this project as a preservation effort, since I have some concerns:
- This SDK documentation could get pulled at anytime. I now have a backup, and can look into hosting solutions if the Segway site goes down.
- The SDK libraries are hosted on JCenter, which has been [deprecated in read-only mode since 2021](https://developer.android.com/build/jcenter-migration). I have a backup of all the SDK libraries, and can re-host them if needed.