# Vicon receiver for ROS 2 (Evanns' Fork for PX4 NED frame and visual odometry EKF fusion)

**ros2-vicon-receiver** is a ROS 2 package, written in C++, that retrieves data from Vicon software and publishes it on ROS 2 topics. The code is partly derived from a mixture of [Vicon-ROS2](https://github.com/aheuillet/Vicon-ROS2) and [Vicon bridge](https://github.com/ethz-asl/vicon_bridge).

This is NOT an official ROS 2 package, but it has been successfully tested with ROS 2 Jazzy Jalisco on Ubuntu 24.04 Noble Numbat and ROS2 Humble Hawksbill on Ubuntu 22.04 Jammy Jellyfish.

---

## Requirements

- [Vicon Tracker](https://www.vicon.com/software/tracker/) running on another machine, with DataStream enabled and reachable over the network (hostname/IP).
- ROS 2 Jazzy Jalisco or Humble Hawksbill installed and sourced (at least *ros-jazzy-ros-base* and *ros-dev-tools* packages, [installation guide](https://docs.ros.org/en/jazzy/Installation.html))
- *rosdep* initialized and updated for managing ROS 2 package dependencies ([installation guide](https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Rosdep.html)) 
- PX4_msgs package (forked minimal version available [here](https://github.com/evannsm/px4_msgs))
- mocap_msgs package (available [here](https://github.com/evannsm/mocap_msgs))

> Note: you do not need system-wide Boost or the Vicon DataStream SDK; both are vendored per-architecture inside this repository.

---

## What's new in this fork

- **ROS 2 Jazzy and Humble** compatibility
- **NED frame:** Transforms Vicon data to North-East-Down (NED) frame convention used by PX4/Aerospace systems
- **Quaternion and Euler Angle outputs:** Publishes vicon NED data in both quaternion and Euler angle formats for flexibility and ease of integration using custom mocap_msgs repository
  - (x, y, z, qw, qx, qy, qz) via `geometry_msgs/msg/PoseStamped` on `<topic>` topics
  - (x, y, z, roll, pitch, yaw) via `vicon_receiver/msg/PoseEuler` on `<topic>_euler` topics
- **PX4 integration:** Publishes `px4_msgs/msg/VehicleOdometry` to `/fmu/in/vehicle_visual_odometry` for direct PX4 autopilot integration

---

## Compatibility

- **ROS 2**: Jazzy Jalisco and Humble Hawksbill
- **OS / arch**: Ubuntu 24.04 and 22.04; `x86_64` and `ARM64` tested
- **Vicon stack**: Vicon Tracker; Vicon DataStream SDK **1.12**
- **SDK & Boost**: vendored per-arch inside this repo (no system install needed)

---

## Package layout & Executable

- Package name: **`vicon_receiver`**
- Primary executable: **`vicon_client`** (C++)
- Launch file: **`client.launch.py`**
- Secondary executables:
  - **'visual_odometry_relay.cpp'**: relays Vicon data to `/fmu/in/vehicle_visual_odometry` in px4_msgs/msg/VehicleOdometry message format PX4 EKF fusion for hardware experiments
  - **'full_state_relay.cpp'**: subscibes to PX4 odometry and local position topics and combines position, velocity, acceleration, and so on into a single `px4_msgs/msg/VehicleFullState` message to be used for logging and a single subscription in PX4 control nodes.
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
    git clone git@github.com:evannsm/ros2-vicon-receiver.git
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

    Notes:

    - `--symlink-install` speeds iteration.
    - `compile_commands.json` appears in the workspace root for IDE/LSP tooling.

6. Source the overlay so your shell sees the built package:

    ```bash
    source install/setup.bash
    ```

7. Alter arguments in the `/launch/client.launch.py` file as needed (e.g., `hostname` to match your Vicon server) and launch
```bash
    ros2 launch vicon_receiver client.launch.py
```

8. Alternatively, you can pass parameters directly on the command line without modifying the launch file (see below for parameter details).
    ```bash
    ros2 launch vicon_receiver client.launch.py   \
      hostname:=192.168.10.1                      \
      topic_namespace:=vicon                      \
      buffer_size:=200                            \
      world_frame:=map                            \
      vicon_frame:=vicon                          \
      map_xyz:='[0.0, 0.0, 0.0]'                  \
      map_rpy:='[0.0, 0.0, 0.0]'                  \
      map_rpy_in_degrees:=false
    ```

9.  After the ros2-vicon-receiver node is running you can:

   - Visualize streamed pose data in RViz2: add a TF display and/or Pose displays for the published topics.
   - Inspect raw messages directly: `ros2 topic list`, then `ros2 topic echo /vicon/<subject_name>/<segment_name>` (adjust topic name as provided by the node).
   - Examine the TF tree: `ros2 run tf2_tools view_frames` (generates frames.pdf) or add the TF display in RViz2 to see live transforms.
   - Record data for later analysis: `ros2 bag record -a` to capture all topics, including the relevant Vicon topics.
   - Monitor performance: `ros2 topic hz /vicon/<subject_name>/<segment_name>` to check update rates.

---

## Node interface

### Launch arguments (convenience wrappers)

| Launch arg           |    Type | Default           | Description                                                            |
| -------------------- | ------: | ----------------- | ---------------------------------------------------------------------- |
| `hostname`           |  string | `192.168.10.1`    | Vicon server hostname/IP                                               |
| `buffer_size`        |     int | `200`             | Internal buffer size used by the Vicon client                          |
| `topic_namespace`    |  string | `vicon`           | Namespace for all published topics                                     |
| `world_frame`        |  string | `map`             | World frame for the tf2 transformation                                 |
| `vicon_frame`        |  string | `vicon`           | Vicon frame for the tf2 transformation                                 |
| `map_xyz`            | list[3] | `[0.0, 0.0, 0.0]` | XYZ translation applied when mapping Vicon frame → World frame         |
| `map_rpy`            | list[3] | `[0.0, 0.0, 0.0]` | Roll-Pitch-Yaw rotation applied when mapping Vicon frame → World frame |
| `map_rpy_in_degrees` |    bool | `false`           | Interpret `map_rpy` as degrees (`true`) or radians (`false`)           |

> These launch arguments pass through to node parameters (see below). Note the node parameter name for the namespace is `namespace`.

### Node parameters (set with `--ros-args -p name:=value`)

| Parameter            |    Type | Default           | Description                                   |
| -------------------- | ------: | ----------------- | --------------------------------------------- |
| `hostname`           |  string | `192.168.10.1`    | Vicon server hostname/IP                      |
| `buffer_size`        |     int | `200`             | Internal buffer size used by the vicon client |
| `namespace`          |  string | `vicon`           | Namespace under which topics are published    |
| `world_frame`        |  string | `map`             | World frame for the tf2 transformation        |
| `vicon_frame`        |  string | `vicon`           | Vicon frame for the tf2 transformation        |
| `map_xyz`            | list[3] | `[0.0, 0.0, 0.0]` | XYZ translation applied for frame mapping     |
| `map_rpy`            | list[3] | `[0.0, 0.0, 0.0]` | RPY rotation applied for frame mapping        |
| `map_rpy_in_degrees` |    bool | `false`           | Interpret `map_rpy` as degrees or radians     |

### Published topics

#### Naming scheme

All topics are published under the configured `namespace` (default `vicon`) and follow this structure:

```text
/<namespace>/<subject_name>/<segment_name>
/<namespace>/<subject_name>/<segment_name>_euler
```

Examples:

- `/vicon/cf0/cf0` (PoseStamped with quaternion)
- `/vicon/cf0/cf0_euler` (PoseEuler with roll/pitch/yaw)

#### Message types

- **Pose per tracked entity:** `geometry_msgs/msg/PoseStamped` (quaternion orientation)
- **Euler angle pose:** `vicon_receiver/msg/PoseEuler` (roll, pitch, yaw in radians)
- **PX4 odometry:** `px4_msgs/msg/VehicleOdometry` published to `/fmu/in/vehicle_visual_odometry`
- **TF:** dynamic transforms for each `<subject_name>_<segment_name>` frame (see TF tree section below).

#### Notes

- `<subject_name>` and `<segment_name>` are taken verbatim from Vicon Tracker.
- Topic frequency depends on your Vicon system and client configuration.

#### QoS

- Standard reliable QoS suitable for state estimation (defaults appropriate for `PoseStamped`).
- PX4 odometry uses BestEffort reliability with TransientLocal durability for PX4 compatibility.

#### Example topic tree

```bash
/vicon/
├── qcar/
│   ├── qcar [geometry_msgs/PoseStamped]
│   └── qcar_euler [vicon_receiver/PoseEuler]
└── cf0/
    ├── cf0 [geometry_msgs/PoseStamped]
    └── cf0_euler [vicon_receiver/PoseEuler]

/fmu/in/vehicle_visual_odometry [px4_msgs/VehicleOdometry]
```

#### Echoing topics

To inspect the raw data from each topic type, use `ros2 topic echo`:

```bash
# Standard PoseStamped (quaternion orientation)
ros2 topic echo /vicon/<subject_name>/<segment_name>

# Euler angles (roll, pitch, yaw)
ros2 topic echo /vicon/<subject_name>/<segment_name>_euler

# PX4 vehicle visual odometry
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

Example with a subject named `cf0`:

```bash
ros2 topic echo /vicon/cf0/cf0
ros2 topic echo /vicon/cf0/cf0_euler
ros2 topic echo /fmu/in/vehicle_visual_odometry
```

## Frames & mapping

- The Vicon frame → world frame mapping is configurable via `world_frame`, `vicon_frame`, `map_xyz` and `map_rpy`.
- `map_rpy_in_degrees` lets you specify rotations in degrees when convenient.
- Frame IDs for subjects/segments are derived from Vicon names.

> Units follow ROS conventions (positions in meters, rotations in radians) in downstream consumers; ensure your system uses consistent units end-to-end.

### TF tree (template + notes)

The package exposes TF for tracked subjects/segments. The expected high-level tree is:

```bash
map (world frame)
└── vicon (vicon_frame)          [static]
    ├── <subject_1>_<segment_1>  [dynamic]
    └── <subject_2>_<segment_2>  [dynamic]
```

#### Static transform (`map → vicon`)

- Defined via node parameters:
  - `world_frame`: name of the world frame (default: `map`)
  - `vicon_frame`: name of the Vicon frame (default: `vicon`)
  - `map_xyz`: translation from `world_frame` to `vicon_frame` in meters (default: `[0.0, 0.0, 0.0]`)
  - `map_rpy`: rotation from `world_frame` to `vicon_frame` in Roll-Pitch-Yaw angles (default: `[0.0, 0.0, 0.0]`)
  - `map_rpy_in_degrees`: `true` if `map_rpy` is given in degrees
- Fixed frame used in RViz: `world_frame` (recommended) or `vicon_frame`

#### Dynamic transforms

- One child frame per `<subject_name>_<segment_name>` populated from Vicon measurements.

#### Tips

- Generate a TF report: `ros2 run tf2_tools view_frames` and inspect the produced PDF.
- Ensure your static `world_frame → vicon_frame` is correct before validating child frames.

---

## Example images

### TF2 frame tree

<img src="docs/images/tf_tree.png" alt="TF2 frame tree for vicon_receiver" width="720">

Example TF tree showing the static `world_frame → vicon_frame` transform and dynamic subject frames.

### RViz: TF and Pose (template)

<img src="docs/images/rviz_tf_pose.png" alt="RViz visualization of TF and PoseStamped" width="720">

RViz view showing TF frames and a Pose display for a tracked subject (e.g., /vicon/qcar/qcar).

---

## Building & linking details

- The package is C++17 and uses `ament_cmake`.
- The **Vicon DataStream SDK (1.12)** and **Boost 1.75** are vendored in `third_party/<arch>/` and linked directly, so you don't need system-wide installations.
- The install step ships the required shared libraries so that runtime lookups succeed without extra `LD_LIBRARY_PATH` setup.

## Troubleshooting / FAQ

**The node can't connect to the Vicon server**: verify `hostname` and network reachability (ping / TCP); check that Vicon Tracker is running and DataStream is enabled.

**Frames look misaligned**: adjust `map_xyz` / `map_rpy` and confirm radians vs degrees via `map_rpy_in_degrees`.

**I don't see TF in RViz**: confirm TF display is enabled and the fixed frame matches your global frame (`world_frame`/`vicon_frame`).

**Which topics are produced?**: topics are under your `namespace` and reflect subject/segment names in Vicon.

---

## License & attribution

- **License:** GNU General Public License v3.0 (GPL-3.0)
- **Current maintainer:** [Andrea Drudi](https://www.unibo.it/sitoweb/andrea.drudi4), [Alice Rosetti](https://www.unibo.it/sitoweb/a.rosetti) and
[Giuseppe Notarstefano](https://www.unibo.it/sitoweb/giuseppe.notarstefano) ([OPT4SMART group](mailto:info@opt4smart.eu)) of the [Department of Electrical, Electronic, and Information Engineering "Guglielmo Marconi" - DEI](https://dei.unibo.it/en) at the University of Bologna, Italy
- **Former contributors:** Andrea Camisa, Andrea Testa
- **Acknowledgments:** Vicon DataStream SDK and community contributors.

---

## Contributing

Issues and PRs are welcome. Please include:

- ROS 2 distro, OS/arch
- Vicon Tracker + SDK versions
- Steps and logs to reproduce

---
