# Vicon4PX4
### A Vicon → ROS 2 → PX4 External Vision Bridge
![Status](https://img.shields.io/badge/Status-Hardware_Validated-blue)
![Vicon](https://img.shields.io/badge/Vicon-Client_3.7-blue)
[![ROS 2 Humble_Compatible](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![ROS 2 Jazzy_Compatible](https://img.shields.io/badge/ROS%202-Jazzy-blue)](https://docs.ros.org/en/jazzy/index.html)
[![PX4 Compatible](https://img.shields.io/badge/PX4-Autopilot-pink)](https://github.com/PX4/PX4-Autopilot)
[![evannsmc.com](https://img.shields.io/badge/evannsmc.com-Project%20Page-blue)](https://www.evannsmc.com/projects/mocap4px4)

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
    git clone git@github.com:evannsm/mocap_px4_relays.git
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
ros2 launch mocap_px4_relays visual_odometry_relay.launch.py
```

And to also include the full state relay:

```bash
ros2 launch mocap_px4_relays full_state_relay.launch.py
```

> **Note:** The relay nodes have been moved to the [`mocap_px4_relays`](../mocap_px4_relays/) package so they can be reused with any motion capture source.

### Example Topic Tree (all three nodes running) assuming your rigid body is named `drone` in the Vicon Tracker app.

```text
/vicon/
├── drone/
│   ├── drone       [geometry_msgs/PoseStamped]
│   └── drone_euler [mocap_msgs/PoseEuler]

/fmu/in/vehicle_visual_odometry          [px4_msgs/VehicleOdometry]   <- to EKF   (mocap_px4_relays)
/merge_odom_localpos/full_state_relay    [mocap_msgs/FullState]       <- from EKF  (mocap_px4_relays)
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

## Relay nodes

The **visual_odometry_relay** and **full_state_relay** nodes have been moved to the [`mocap_px4_relays`](../mocap_px4_relays/) package so they can be reused with any motion capture source (Vicon, OptiTrack, etc.). See that package's README for full documentation.

### Data pipeline

```text
Vicon Tracker (ENU, millimeters)
        |
   vicon_client node  (vicon4px4)
        |  converts ENU -> NED, mm -> m
        v
  /vicon/*rigid_body_name*/*rigid_body_name* (geometry_msgs/PoseStamped, NED)
        |
  visual_odometry_relay node  (mocap_px4_relays)
        |  reorders quaternion, stamps, publishes at 35 Hz
        v
  /fmu/in/vehicle_visual_odometry  (px4_msgs/VehicleOdometry)
        |
   PX4 EKF2 (fuses vision + IMU)
        |
        v
  /fmu/out/vehicle_odometry & /fmu/out/vehicle_local_position
        |
  full_state_relay node  (mocap_px4_relays)  [optional]
        |  merges both into one topic at 40 Hz
        v
  /merge_odom_localpos/full_state_relay  (mocap_msgs/FullState)
```

For the EKF to accept vision input you must enable it on the PX4 side (the `EKF2_EV_CTRL` and `EKF2_HGT_REF` must be set up according to what your motion capture system can provide). It is also recommended to turn off magnetometer fusion to avoid issues in indoor environments.

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
│   └── utils.cpp                   # frame conversion utilities
├── launch/
│   ├── client.launch.py                      # vicon_client only
│   ├── client_and_visual_odometry.launch.py  # client + relay (uses mocap_px4_relays)
│   └── client_vision_full_all.launch.py      # all three nodes (uses mocap_px4_relays)
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


## Website

Part of the [**evannsmc** open-source portfolio](https://www.evannsmc.com/projects) — [project page →](https://www.evannsmc.com/projects/mocap4px4)

## License & attribution

- **License:** GNU General Public License v3.0 (GPL-3.0)
