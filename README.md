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

## Driving: push to start and the stop path

The Loomo's screen shows one large red button, styled as an emergency stop, and the driving state
in large letters. `DriveGate.kt` holds the logic; `DriveGateTest.kt` covers it.

- **The app starts NOT DRIVING.** `/cmd_vel` reaches the wheels only while DRIVING. A tap on the
  button starts driving; pressing it again stops at once (on the press, not the release). No topic
  can switch the app to driving.
- **A tap cannot start driving without the Jetson's heartbeat** (`/loomo/jetson_watchdog`, sent
  every 0.5 s). The screen then reads "No Jetson heartbeat: cannot start".
- **NOT DRIVING keeps the base balancing at zero speed.** The base stays in raw mode; the app only
  ever sets its velocities to zero. It never changes the control mode or turns balancing off.
- **No `/cmd_vel` for 0.5 s: the wheels are set to zero.** Driving stays on, and the next
  `/cmd_vel` moves them again. A released teleop stick or a stopped planner ends like this.
- **No heartbeat for 1.5 s: the app stops driving and stays stopped** (latched). It does not
  resume when the heartbeat returns; only a tap re-arms it. The head light still turns red as before.
- **The screen leaves the front: the app stops driving at once, latched the same way.** Another app
  or a system dialog in front, or the screen turned off (the activity pauses), means the button
  can't be reached. Coming back in front does not re-arm; a tap does.
- The timeouts are checked every 20 ms and before every `/cmd_vel`, so they act within 20 ms of
  their limit. A zero the base does not take is retried every 20 ms until it does.
- When the base service binds, and when the app stops or restarts, the app sends a zero.
- The screen stays on while driving, so the first touch always reaches the button and a screen
  timeout cannot stop a drive. The power button still turns it off, and that stops driving.

The SDK does not stop the base by itself: in raw mode `Base.setLinearVelocity` and
`Base.setAngularVelocity` (basesdk 0.6.1653) each pass one value over binder to Segway's
locomotion service and return. The SDK keeps no timer and resends nothing, and `Base.stop()` is
only both velocities set to zero. Before this gate, the app forwarded every `/cmd_vel` and sent
nothing when they stopped.

| Topic | Type | QoS | Meaning |
|---|---|---|---|
| `/loomo/drive_enabled` | `std_msgs/Bool` | reliable, transient local, depth 1 | `true` while DRIVING. Published on every change and again every second. |

The app only publishes this topic; it subscribes to nothing that can start driving. A reader
behind a DDS bridge gets the current value within a second even if the bridge does not keep
transient-local samples.

Logcat, tag `DriveGate`: one line per start, stop (with its reason), refused start, `/cmd_vel`
timeout (with the last command), heartbeat loss and return. `StreamStats` adds a line every 5 s:

```
adb logcat -s DriveGate:* StreamStats:I
drive mode=NOT_DRIVING reason=START_UP heartbeat=ok following=false cmd passed=0 ignored=0 invalid=0 timeouts=0 starts=0 refused=0 stops button=0 heartbeat=0 hidden=0 zero_refused=0
```

## Camera streams

The app publishes the RealSense depth and colour cameras and the fisheye camera at 30 fps on
ROS domain 1 (see `MainActivity.kt`; the Jetson's DDS-Router bridges domain 1 to domain 0):

| Stream | Topics (default) | Frame |
|---|---|---|
| depth | `/loomo/realsense/depth/image_depth_rect` (`16UC1`), `/loomo/realsense/depth/camera_info` | 320x240, 154 KB |
| colour | `/loomo/realsense/color/image_color/h264` (`CompressedImage`, H.264), `/loomo/realsense/color/camera_info` | 640x480, 15-85 KB |
| fisheye | `/loomo/fisheye/image` (`mono8`), `/loomo/fisheye/camera_info` | 640x480, 307 KB |

All image and camera_info topics are RELIABLE with a keep-last history of one second (two for
H.264), published asynchronously. The Vision SDK callback only copies the frame into a bounded
per-stream queue; a worker thread publishes. When a worker falls more than `queue.depth` frames
behind, the incoming frame is dropped, counted and logged at error level with the queue state.
Nothing is ever skipped silently: every 5 s a `StreamStats` logcat line per stream reports the
frames the vision service delivered (and the ones it numbered but never delivered), queued,
dropped, published per topic and failed:

```
adb logcat -s StreamStats:* BridgeConfig:* MessageWarmup:*
```

By default `header.stamp.nanosec % 1000` carries the vision service's frame number modulo 1000
on every image and camera_info message (the stamp is truncated to the microsecond, so those
digits are otherwise zero; the stamp is perturbed by under a microsecond). A receiver can check
for missing frames without any extra topic. Set `stats.tagFrameNum=false` to turn it off.

### Time stamps

Every `header.stamp` is on the **Jetson's clock**, not the Loomo's. The Segway services stamp
frames, `Pose2D`, IMU and TF data in microseconds of the Loomo's wall clock, which has no internet
time source and ran ~652 s ahead of the Jetson (and moves at every reboot, when it is re-read
from the RTC). `TimeSync.kt` queries the SNTP responder on the Jetson's end of the Loomo link
(`10.0.0.1:123`, loomo_jetson_config's `loomo-sntp` service) once a second, keeps the answer
with the shortest round trip of the last 16, and adds that offset to every stamp. Neither clock
is changed: the app cannot set the Loomo's clock without root, and stepping it would disturb the
vision service, which drops frames whose timestamps do not increase.

- Until the first answer the app publishes **no stamped message** (TF, odometry, joint states,
  images, camera_info, ranges); listeners that once see a stamp 650 s in the future reject the
  correct ones afterwards. The watchdog heartbeat and `/cmd_vel` are unaffected. The first answer
  normally arrives 3-6 ms after ROS starts; without one, `TimeSync` logs an error every 30 s.
- Measured on 2026-10-13: round trip ~0.6 ms (offset error under 0.3 ms), Loomo crystal drift
  -17 to -27 ppm against the Jetson (under 0.5 ms across the 16 s window). On the Jetson,
  receive time minus stamp is 14-60 ms for camera_info, odometry, TF and joint states and
  ~100 ms for H.264 (it was -652 400 ms), and L2 LIDAR clouds transform into `loomo_odom` at
  their own stamps.
- A frame's camera_info, image and H.264 messages share one converted stamp.
- The sync is not the limit on stamp accuracy; the Segway stamps are. The fisheye's
  `platformTimeStamp` is the least consistent: right after a start some fisheye frames are only
  5 ms old when they reach the app (depth and colour: 21-28 ms), and within one session its
  stamp-to-arrival time shifted by ~35 ms. Once, shortly after a start, a fisheye camera_info
  arrived on the Jetson 19 ms *before* its stamp; not seen again in four more restarts.
- `adb logcat -s TimeSync:*` prints a line a minute: offset, round trips, drift, steps of either
  clock, and how old each SDK stamp source is when converted (all small and positive, which is
  how the sources were shown to be on the Loomo's wall clock).

### TF, odometry and joint states

One thread (`tf-publisher`) does two jobs. Per depth frame (30 Hz) it looks up the head and
camera transforms and the joint states at the frame's time. On its own 20 Hz timer it reads Pose2D
and the base IMU, which the Segway services update at 20 Hz (50 ms apart, each at its own phase;
the SDK only returns the latest snapshot). So the odometry and the tilt keep coming when the depth
stream stalls or is disabled; before, they were read per depth frame and stopped with it. Each SDK
sample is published once, decided by its raw timestamp (`SampleGate.kt`):

| Topic / transform | Rate | Stamp | Driven by |
|---|---|---|---|
| `/tf` `base_link -> neck_center_body_yaw_frame`, the camera frames and their optical frames | 30 Hz | depth frame | depth frame |
| `/tf` `loomo_odom -> base_odom` (only with `tf.odom=true`), `/loomo/odom` | 20 Hz | Pose2D sample | 20 Hz timer |
| `/tf` `base_odom -> base_link`, ultrasonic and IR mounts | 20 Hz | base IMU sample | 20 Hz timer |
| `/loomo/joint_states` (`neck_yaw_joint`, `head_pitch_joint`) | 30 Hz | depth frame | depth frame |
| `/loomo/head/world_imu`, `/loomo/head/joint_roll` (only with `head.imu=true`, the default; see Head attitude) | 20 Hz | head sample | 20 Hz timer |

`base_odom -> base_link` is the base's pitch and roll. Segway's locomotion service converts the
base controller's angles, counted in 0.01 degree, with `raw * PI / 32767`. BASE_IMU therefore
reports 18000/32767 = 0.549 times the true angles, and its roll has the opposite sign to ROS roll.
The app publishes pitch = `floatData[0] * 32767/18000` and roll = `-floatData[1] * 32767/18000`
(`BaseTilt.kt`). Before, `base_link` leaned about half as far as the body: on one Loomo
(2026-10-23) the reported pitch was 0.525 times and the roll -0.520 times the tilt of gravity
measured by a separate IMU.

Before, every capture republished the latest snapshot: ~33% of the odometry, `loomo_odom ->
base_odom` and `base_odom -> base_link` messages repeated the previous sample, and some copies
differed by a fraction of a millisecond after the clock-offset conversion. The joint states took
the neck yaw at the depth frame's time but the stamp and the head pitch from the 20 Hz head IMU, so
~32% repeated a stamp with different content and tf2 dropped robot_state_publisher's head transforms
for them (`TF_REPEATED_DATA`). Both joints now come from one TF lookup at the depth frame's time;
the pitch is the joint angle relative to the body (the head IMU's pitch was relative to the world).

The odometry twist is, by default, the Pose2D difference over 0.25 s (`PoseDiffTwist.kt`). The
SDK's own velocities are low-pass filtered (time constant ~0.45 s): against the L2 LIDAR's gyro on a
driven circuit the SDK's angular velocity lagged 395 ms (RMS error 0.062 rad/s), the 0.25-0.30 s
pose difference 160-190 ms (0.049-0.050 rad/s). Shorter windows are worse, because the pose is
quantised (yaw in 0.0193 rad encoder ticks, position in 4.45 mm steps). The SDK's values are used
until the window has filled and after a Pose2D gap of more than 0.6 s. A Pose2D stream stamp that
jumps back by more than a second (the Loomo's clock was set back) restarts both the dedupe and the
window.

`StreamStats` adds a line with the capture queue, the new and repeated SDK samples, the twist
fallbacks and the 20 Hz timer's ticks: `tf queue depth=0 drops=0 pose2d new=... repeat=...
base_imu new=... repeat=... twist=pose_diff sdk_fallbacks=... odom_tf=false base_ticks=...
skipped=... joint_state_skips=... head_imu new=... repeat=... head_roll new=... repeat=...
head_fail=...`. With the timer at the sources' rate, `pose2d new` should grow by about 100 every 5 s;
`skipped` counts timer ticks lost to a stalled thread; `head_fail` counts failed head queries or
publishes.

### Head attitude

The head has its own IMU. On the same 20 Hz timer the app reads two of the head's sensors and
publishes them. It only reads them: nothing here commands the head.

| Topic | Type | Content |
|---|---|---|
| `/loomo/head/world_imu` | `sensor_msgs/Imu`, frame `head_pitch_link` | `HEAD_WORLD_IMU`: the head's attitude in the world, as the orientation only. Angular velocity and linear acceleration are not provided (covariance element 0 is -1); the orientation covariance is unknown (all zeros) |
| `/loomo/head/joint_roll` | `sensor_msgs/JointState`, one joint `head_roll_joint` | `HEAD_JOINT_ROLL`: the head's roll joint. The URDF has no such joint, so it is not in `/loomo/joint_states`; a `JointState` rather than a `std_msgs/Float64` so the sample keeps its stamp |

Each sample is published once, stamped with its own SDK time on the Jetson's clock. The head's
angles need no scale correction: Segway's service converts them at 182 counts per degree, which is
how the head encodes them (BASE_IMU's factor does not apply). The conversion is in `HeadAttitude.kt`:

| ROS | From | Sign | How sure |
|---|---|---|---|
| pitch (nose down positive) | `-floatData[0]` | Segway's head pitch is positive nose up | Indirect evidence: `HEAD_JOINT_PITCH`, converted the same way, read -0.111 rad when Segway's TF had the joint 0.112 rad nose down; in a 2026-10-13 recording `floatData[0]` moved against the base's pitch. Check on the robot |
| roll (left side up positive) | `+floatData[1]` | passed through | A guess (the right-handed set that matches the pitch and yaw signs). No recording has it yet. Check on the robot |
| yaw (counterclockwise positive) | `+floatData[2]` | the service already negates it, as it does the joint yaw | Inferred from the joint yaw's checked sign; arbitrary zero |
| joint roll (left side up positive) | `+HEAD_JOINT_ROLL floatData[0]` | passed through | A guess, as for the roll |

The angles are taken as yaw, then pitch, then roll (tf2's `setRPY`); Segway does not document the
order. Near level it barely matters (0.08 degree of tilt at 10 degrees of pitch and 5 of roll).
`head.imu=false` turns both topics off and stops the head queries.

### Configuration

Settings are read once at start-up from `bridge.properties` in the app's external files
directory, then overridden by the extras of the starting intent. The effective values are logged
under `BridgeConfig`.

```
adb push bridge.properties /sdcard/Android/data/com.autom8ed.lr2/files/bridge.properties
adb shell am start -n com.autom8ed.lr2/.MainActivity --es colour.raw true   # one-off override
```

| Key | Default | Meaning |
|---|---|---|
| `depth.enabled`, `colour.enabled`, `fisheye.enabled` | `true` | run the stream |
| `depth.raw`, `fisheye.raw` | `true` | publish the raw `sensor_msgs/Image` |
| `colour.raw` | `false` | raw colour is 1.2 MB per frame (37 MB/s at 30 fps); the USB 2.0 link cannot carry it next to depth and fisheye |
| `colour.h264` | `true` | the H.264 stream (hardware encoder, 4 Mbit/s, I-frame every second) |
| `queue.depth` | `8` | frames of jitter a worker may fall behind before frames are dropped |
| `stats.tagFrameNum` | `true` | frame number in the stamp, see above |
| `stats.periodS` | `5` | seconds between `StreamStats` lines |
| `clock.sync` | `true` | stamp on the Jetson's clock (see Time stamps); `false` keeps the Loomo's own clock, as before |
| `clock.server` | `10.0.0.1` | the SNTP server to measure against |
| `odom.twist` | `pose_diff` | `/loomo/odom` twist from the Pose2D difference over `odom.twist_window_s` (see TF, odometry and joint states); `sdk` publishes the SDK's low-pass filtered velocities, as before |
| `odom.twist_window_s` | `0.25` | the `pose_diff` window in seconds, 0.05 to 0.5 |
| `head.imu` | `true` | publish `/loomo/head/world_imu` and `/loomo/head/joint_roll` from the head's own sensors (see Head attitude); `false` stops the head queries |
| `tf.odom` | `false` | publish `loomo_odom -> base_odom` (the Pose2D odometry) on `/tf`. Off by default: an odometry filter on the ROS side fuses `/loomo/odom` and owns `base_odom`'s parent (a frame has only one). Set `true` when nothing else publishes it |
| `dds.priority` | `true` | load `assets/fastdds_profiles.xml`: TF, `/loomo/odom`, `/loomo/joint_states` and the two head topics get their own Fast DDS sender thread instead of queueing behind the raw images (#17); `false` runs with Fast DDS defaults |

## Preservation
The Loomo was discontinued at some point between 2019 and 2024. For now, the SDK documentation is still available [here](https://developer.segwayrobotics.com/developer/documents/segway-robots-sdk.html), and the SDK libraries can still be downloaded from the online Maven repositories.

I'm partially working on this project as a preservation effort, since I have some concerns:
- This SDK documentation could get pulled at anytime. I now have a backup, and can look into hosting solutions if the Segway site goes down.
- The SDK libraries are hosted on JCenter, which has been [deprecated in read-only mode since 2021](https://developer.android.com/build/jcenter-migration). I have a backup of all the SDK libraries, and can re-host them if needed.