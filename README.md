# Vicon4PX4
### A Vicon → ROS 2 → PX4 External Vision Bridge

`vicon4px4` is a ROS 2 (C++) package that streams Vicon motion capture data into the PX4 EKF using External Vision fusion, enabling position and heading fusion for hardware flight experiments. The package converts Vicon ENU measurements to NED, and publishes pose data in quaternions as well as euler angles under the rigid body name as defined in Vicon Tracker.

Another launch file then relays this information for use in PX4 External Vision EKF Fusion by publishing the data as `px4_msgs/msg/VehicleOdometry` messages on `/fmu/in/vehicle_visual_odometry`, and handles the quaternion reordering and timestamping that PX4 expects. A secondary full-state relay node is available to merge the fused EKF output back from `/fmu/out/vehicle_odometry`, and `/fmu/out/vehicle_local_position` into one topic that relays all pose data including available higher order derivatives from the EKF for convenient logging and control.

Derived from [ROS2-Vicon-Receiver](https://github.com/OPT4SMART/ros2-vicon-receiver). Tested with ROS 2 Jazzy Jalisco (Ubuntu 24.04) and Humble Hawksbill (Ubuntu 22.04).

---

## Quick start

Assumes ROS 2 Jazzy/Humble is already installed and `rosdep` initialized.

1. Source ROS 2:

    ```bash
    source /opt/ros/$ROS_DISTRO/setup.bash
    ```

2. Create (or choose) a workspace directory:

    ```bash
    mkdir -p ~/ws_mocap_px4_msgs_drivers/src
    cd ~/ws_mocap_px4_msgs_drivers/src
    ```

3. Clone the packages into `src/`:

    ```bash
    git clone git@github.com:evannsm/vicon4px4.git
    git clone -b v1.16_minimal_msgs git@github.com:evannsm/px4_msgs.git
    git clone git@github.com:evannsm/mocap_msgs.git
    cd ..   # back to workspace root
    ```

4. Install ROS 2 dependencies (none of the vendored Boost / Vicon SDK need system install):

    ```bash
    rosdep install --from-paths src --rosdistro $ROS_DISTRO -y --ignore-src
    ```

5. Build with colcon (Python invocation helps with virtual environments):

    ```bash
    python3 -m colcon build                 \
      --symlink-install                     \
      --cmake-args                          \
        -DCMAKE_BUILD_TYPE=Release          \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
    ```

6. Source the overlay:

    ```bash
    source install/setup.bash
    ```

### Setting up Networking Parameters:
1. Navigate to `vicon4px4/launch/client.launch.py` and set the hostname default value to the static IP address of the desktop that runs the Vicon Tracker software on a local area network (LAN). For instance, the desktop running the vicon tracker could have an IP on the LAN of `192.168.10.2' as shown below:
```python
    hostname_arg = DeclareLaunchArgument(
        'hostname',
        default_value='192.168.10.2',
        description='Vicon server hostname or IP address'
    )
```

2. Make sure the Ubuntu computer that runs this ROS2 stack has a static IP on the same LAN as the Vicon Tracker computer.

    
### Launching for vision fusion

To run the Vicon client and visual odometry relay (the typical flight-test configuration):

In one terminal:
```bash
ros2 launch vicon4px4 client.launch.py
```

In another terminal:
```bash
ros2 launch vicon4px4 visual_odometry_relay.launch.py
```

And to also include the full state relay:

```bash
ros2 launch vicon4px4 full_state_relay.launch.py
```

### Example Topic Tree (all three nodes running) assuming your rigid body is named `drone` in the Vicon Tracker app.

```text
/vicon/
├── drone/
│   ├── drone       [geometry_msgs/PoseStamped]
│   └── drone_euler [mocap_msgs/PoseEuler]

/fmu/in/vehicle_visual_odometry          [px4_msgs/VehicleOdometry]   <- to EKF
/merge_odom_localpos/full_state_relay    [mocap_msgs/FullState]       <- from EKF
```

```bash
# Verify vision data is reaching PX4
ros2 topic echo /fmu/in/vehicle_visual_odometry

# Check the merged full-state output
ros2 topic echo /merge_odom_localpos/full_state_relay

# Raw Vicon pose
ros2 topic echo /vicon/drone/drone
```

#### Published topics of the client.launch.py alone

All topics are published under the configured `namespace` (default `vicon`):

```text
/<namespace>/<subject_name>/<segment_name>         [geometry_msgs/PoseStamped]
/<namespace>/<subject_name>/<segment_name>_euler    [mocap_msgs/PoseEuler]
```

- **PoseStamped**: position (x, y, z) + quaternion (qw, qx, qy, qz) in NED
- **PoseEuler**: position (x, y, z) + roll, pitch, yaw in radians

`<subject_name>` and `<segment_name>` are taken verbatim from Vicon Tracker.

#### TF tree

```text
map (world_frame)
└── vicon (vicon_frame)          [static]
    ├── <subject_1>_<segment_1>  [dynamic]
    └── <subject_2>_<segment_2>  [dynamic]
```

The static `map -> vicon` transform is defined by `map_xyz` and `map_rpy`. Dynamic child frames update with each Vicon measurement.
 Published topics

All topics are published under the configured `namespace` (default `vicon`):

```text
/<namespace>/<subject_name>/<segment_name>         [geometry_msgs/PoseStamped]
/<namespace>/<subject_name>/<segment_name>_euler    [mocap_msgs/PoseEuler]
```

- **PoseStamped**: position (x, y, z) + quaternion (qw, qx, qy, qz) in NED
- **PoseEuler**: position (x, y, z) + roll, pitch, yaw in radians

`<subject_name>` and `<segment_name>` are taken verbatim from Vicon Tracker.

---

## Visual Odometry Relay

The **`visual_odometry_relay`** node is the bridge between your Vicon data and the PX4 EKF. It:

- Subscribes to `/vicon/drone/drone` (`geometry_msgs/msg/PoseStamped`) containing the NED-converted pose from `vicon_client`
- Converts the ROS quaternion order (x, y, z, w) to PX4 order (w, x, y, z)
- Sets the `pose_frame` field to `POSE_FRAME_NED` so EKF2 knows the coordinate convention
- Timestamps each message in microseconds using the ROS clock
- Publishes at a steady **35 Hz** on `/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`) in order to avoid overwhelming the PX4 EKF and causing the board to brown out

The fixed-rate timer decouples publishing from the Vicon callback rate; the relay always sends the most recent pose, which keeps the EKF input smooth even if individual Vicon frames arrive with jitter.


### How external vision fusion works

The PX4 EKF2 can fuse external position and attitude measurements (from a mocap system, VIO camera, etc.) alongside its IMU to produce smooth, drift-corrected state estimates. This package provides the complete pipeline from Vicon to EKF input:

```text
Vicon Tracker (ENU, millimeters)
        |
   vicon_client node
        |  converts ENU -> NED, mm -> m
        v
  /vicon/*rigid_body_name*/*rigid_body_name* & /vicon/*rigid_body_name*/*rigid_body_name_euler* (geometry_msgs/PoseStamped, NED)
        |
  visual_odometry_relay node
        |  reorders quaternion (x,y,z,w -> w,x,y,z)
        |  stamps with PX4-compatible microsecond timestamp
        |  publishes at fixed 35 Hz
        v
  /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)
        |
   PX4 EKF2 (fuses vision + IMU)
        |
        v
  /fmu/out/vehicle_odometry          (fused position, velocity, attitude, angular rate)
  /fmu/out/vehicle_local_position    (fused position, velocity, acceleration)
        |
  full_state_relay node  [optional]
        |  merges both into one topic at 40 Hz
        v
  /merge_odom_localpos/full_state_relay  (mocap_msgs/FullState)
```

For the EKF to accept vision input you must enable it on the PX4 side (the `EKF2_EV_CTRL` and `EKF2_HGT_REF` must be set up according to what your motion capture system can provide). It is also recommended to turn off magnetometer fusion to avoid issues in indoor environments.

---


## Full state relay (secondary)

The **`full_state_relay`** node is an optional convenience for downstream control and logging nodes. Instead of subscribing to two separate PX4 topics, you get one merged message containing position, velocity, acceleration, quaternion, and angular velocity.

It subscribes to the fused EKF outputs:

| Source topic                        | Data pulled                                      |
| ----------------------------------- | ------------------------------------------------ |
| `/fmu/out/vehicle_odometry`         | position, velocity, quaternion, angular velocity  |
| `/fmu/out/vehicle_local_position`   | acceleration (ax, ay, az)                         |

And publishes `mocap_msgs/msg/FullState` on `/merge_odom_localpos/full_state_relay` at **40 Hz**.

A gating mechanism prevents publishing stale or low-rate data:

- Both source callbacks must be arriving at >= `min_rate_hz` (default 50 Hz), tracked via exponential moving average
- Both must have received data within `recent_timeout_sec` (default 100 ms)

| Parameter            |  Type  | Default | Description                              |
| -------------------- | -----: | ------- | ---------------------------------------- |
| `min_rate_hz`        | double | `50.0`  | Minimum acceptable callback rate         |
| `recent_timeout_sec` | double | `0.10`  | Maximum age of data before it's stale    |
| `rate_ema_alpha`     | double | `0.9`   | EMA smoothing factor (higher = smoother) |

---

## Requirements

- [Vicon Tracker](https://www.vicon.com/software/tracker/) running on another machine, with DataStream enabled and reachable over the network (hostname/IP)
- ROS 2 Jazzy Jalisco or Humble Hawksbill installed and sourced (at least *ros-jazzy-ros-base* and *ros-dev-tools* packages, [installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- *rosdep* initialized and updated for managing ROS 2 package dependencies ([installation guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html))
- PX4_msgs package (forked minimal version available [here](https://github.com/evannsm/px4_msgs))
- mocap_msgs package (available [here](https://github.com/evannsm/mocap_msgs))

> Note: you do not need system-wide Boost or the Vicon DataStream SDK; both are vendored per-architecture inside this repository.

---

## Compatibility

- **ROS 2**: Jazzy Jalisco and Humble Hawksbill
- **OS / arch**: Ubuntu 24.04 and 22.04; `x86_64` and `ARM64` tested
- **Vicon stack**: Vicon Tracker; Vicon DataStream SDK **1.12**
- **SDK & Boost**: vendored per-arch inside this repo (no system install needed)

---

## Package layout

```text
vicon4px4/
├── src/
│   ├── communicator.cpp            # vicon_client - connects to Vicon, converts ENU->NED
│   ├── publisher.cpp               # per-subject publisher creation
│   ├── utils.cpp                   # frame conversion utilities
│   ├── visual_odometry_relay.cpp   # relays NED pose to PX4 EKF (primary)
│   └── full_state_relay.cpp        # merges EKF outputs into FullState (secondary)
├── launch/
│   ├── client.launch.py                      # vicon_client only
│   ├── visual_odometry_relay.launch.py       # relay only
│   ├── full_state_relay.launch.py            # full state only
│   ├── client_and_visual_odometry.launch.py  # client + relay (typical flight config)
│   └── client_vision_full_all.launch.py      # all three nodes
└── third_party/                              # vendored Boost 1.75 & Vicon SDK 1.12
```

---

## Building & linking details

- The package is C++17 and uses `ament_cmake`.
- The **Vicon DataStream SDK (1.12)** and **Boost 1.75** are vendored in `third_party/<arch>/` and linked directly, so you don't need system-wide installations.
- The install step ships the required shared libraries so that runtime lookups succeed without extra `LD_LIBRARY_PATH` setup.

## Troubleshooting / FAQ

**EKF not fusing vision data**: Ensure `EKF2_EV_CTRL` is set to enable position and/or yaw fusion from external vision. Check that `/fmu/in/vehicle_visual_odometry` is being published at the expected rate with `ros2 topic hz`.

**The node can't connect to the Vicon server**: verify `hostname` and network reachability (ping / TCP); check that Vicon Tracker is running and DataStream is enabled.

**Frames look misaligned**: adjust `map_xyz` / `map_rpy` and confirm radians vs degrees via `map_rpy_in_degrees`.

**Full state relay not publishing**: the gating requires both `/fmu/out/vehicle_odometry` and `/fmu/out/vehicle_local_position` to arrive at >= 50 Hz. Confirm PX4 is running and the DDS/uXRCE bridge is healthy.

**I don't see TF in RViz**: confirm TF display is enabled and the fixed frame matches your global frame (`world_frame`/`vicon_frame`).

---


### Frames & mapping

- The Vicon frame -> world frame mapping is configurable via `world_frame`, `vicon_frame`, `map_xyz` and `map_rpy`.
- `map_rpy_in_degrees` lets you specify rotations in degrees when convenient.
- Frame IDs for subjects/segments are derived from Vicon names.

> Units follow ROS conventions (positions in meters, rotations in radians) in downstream consumers; ensure your system uses consistent units end-to-end.

#### TF tree

```text
map (world_frame)
└── vicon (vicon_frame)          [static]
    ├── <subject_1>_<segment_1>  [dynamic]
    └── <subject_2>_<segment_2>  [dynamic]
```

The static `map -> vicon` transform is defined by `map_xyz` and `map_rpy`. Dynamic child frames update with each Vicon measurement.

---

## Example images

### TF2 frame tree

<img src="docs/images/tf_tree.png" alt="TF2 frame tree for vicon4px4" width="720">

Example TF tree showing the static `world_frame -> vicon_frame` transform and dynamic subject frames.

### RViz: TF and Pose

<img src="docs/images/rviz_tf_pose.png" alt="RViz visualization of TF and PoseStamped" width="720">

RViz view showing TF frames and a Pose display for a tracked subject.

---


## License & attribution

- **License:** GNU General Public License v3.0 (GPL-3.0)
