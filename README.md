# ROS2 Humble — Gahan AI Assignment
### Tasks: Differential Drive Robot Simulation with Nav2 + Custom ROS2 Interfaces

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Workspace Structure](#2-workspace-structure)
3. [Package Overview](#3-package-overview)
4. [Task 1 — Robot Simulation, Sensors & Nav2](#4-task-1--robot-simulation-sensors--nav2)
   - 4.1 [URDF Robot Description](#41-urdf-robot-description)
   - 4.2 [Custom Gazebo World](#42-custom-gazebo-world)
   - 4.3 [Sensor Configuration](#43-sensor-configuration)
   - 4.4 [Build & Launch Instructions](#44-build--launch-instructions)
   - 4.5 [SLAM — Building the Map](#45-slam--building-the-map)
   - 4.6 [Nav2 — Autonomous Navigation](#46-nav2--autonomous-navigation)
5. [Task 2 — Custom Messages, Services & Nodes](#5-task-2--custom-messages-services--nodes)
   - 5.1 [Custom Message — Person.msg](#51-custom-message--personmsg)
   - 5.2 [Custom Service — Value.srv](#52-custom-service--valuesrv)
   - 5.3 [Publisher & Subscriber](#53-publisher--subscriber)
   - 5.4 [Service Server & Client](#54-service-server--client)
   - 5.5 [Build & Run Instructions](#55-build--run-instructions)
6. [ROS2 Topics Reference](#6-ros2-topics-reference)
7. [TF Frame Tree](#7-tf-frame-tree)
8. [Media Proof Index](#8-media-proof-index)
9. [Troubleshooting](#9-troubleshooting)

---

## 1. System Requirements

| Component | Requirement |
|---|---|
| **Operating System** | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| **ROS2 Distribution** | **Humble Hawksbill** (ROS2 Humble) |
| **Gazebo** | Gazebo Classic 11 (`gazebo_ros_pkgs`) |
| **Python** | Python 3.10+ |
| **CMake** | 3.8+ |
| **RAM** | 8 GB minimum (16 GB recommended for Gazebo + Nav2) |
| **GPU** | Recommended for Gazebo rendering; integrated GPU is sufficient |

### Required ROS2 Packages

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-xacro \
  ros-humble-rviz2 \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-nav2-common \
  ros-humble-nav2-map-server \
  ros-humble-teleop-twist-keyboard
```

---

## 2. Workspace Structure

```
ros2_ws/
├── src/
│   ├── robot_description/          # URDF/xacro model + RViz config
│   │   ├── urdf/
│   │   │   └── robot.urdf.xacro    # Full robot model with sensors & plugins
│   │   └── rviz/
│   │       └── robot.rviz          # RViz config for robot visualisation
│   │
│   ├── robot_bringup/              # Gazebo world + simulation launch files
│   │   ├── launch/
│   │   │   ├── simulation.launch.py  # Core: Gazebo + RSP + spawn (no RViz)
│   │   │   └── gazebo.launch.py      # Full: simulation + RViz
│   │   └── worlds/
│   │       └── training_world.world  # Custom Gazebo world with obstacles
│   │
│   ├── robot_navigation/           # SLAM, Nav2, maps, configs
│   │   ├── config/
│   │   │   ├── slam_params.yaml    # SLAM Toolbox parameters
│   │   │   └── nav2_params.yaml    # Nav2 stack parameters (AMCL, planners, etc.)
│   │   ├── launch/
│   │   │   ├── slam.launch.py        # Simulation + SLAM for map building
│   │   │   └── navigation.launch.py  # Simulation + Nav2 for goal navigation
│   │   ├── maps/
│   │   │   ├── map.yaml            # Saved map metadata
│   │   │   └── map.pgm             # Saved occupancy grid image
│   │   └── rviz/
│   │       └── nav2.rviz           # RViz config for SLAM / Nav2
│   │
│   ├── training_interfaces/        # Custom messages and services (C++)
│   │   ├── msg/
│   │   │   └── Person.msg          # name (string) + age (int32)
│   │   └── srv/
│   │       └── Value.srv           # a, b (int32) → val (int32)
│   │
│   └── training/                   # Publisher, subscriber, server, client (Python)
│       └── training/
│           ├── person_publisher.py
│           ├── person_subscriber.py
│           ├── service_server.py
│           └── service_client.py
│
└── Media_Proof/
    ├── Folder_Structure.png
    ├── Task1_Nav2/
    │   ├── Screen Recording/       # .webm videos
    │   └── Screenshots/            # Organised by category
    └── Task2_ROS_Programming/      # Screenshots of running nodes
```

---

## 3. Package Overview

| Package | Type | Purpose |
|---|---|---|
| `robot_description` | CMake | URDF model — robot geometry, links, joints, sensor plugins |
| `robot_bringup` | CMake | Gazebo simulation launch, custom `.world` file |
| `robot_navigation` | CMake | SLAM Toolbox + Nav2 launch, params, saved map |
| `training_interfaces` | CMake | Custom `Person.msg` and `Value.srv` definitions |
| `training` | Python (ament) | Publisher, subscriber, service server, service client nodes |

---

## 4. Task 1 — Robot Simulation, Sensors & Nav2

### 4.1 URDF Robot Description

**File:** `src/robot_description/urdf/robot.urdf.xacro`

The robot is a **2-wheel differential drive** platform built entirely in URDF/Xacro.

#### Physical Model

| Link | Geometry | Dimensions | Mass |
|---|---|---|---|
| `base_link` | Box (chassis) | 0.35 × 0.25 × 0.12 m | 2.2 kg |
| `left_wheel` | Cylinder | r = 0.09 m, l = 0.05 m | 0.35 kg |
| `right_wheel` | Cylinder | r = 0.09 m, l = 0.05 m | 0.35 kg |
| `caster_wheel` | Sphere | r = 0.04 m | 0.15 kg |
| `imu_link` | Box | 0.04 × 0.04 × 0.02 m | 0.01 kg |
| `laser_link` | Cylinder | r = 0.045 m, l = 0.05 m | 0.2 kg |

#### Coordinate Model

```
base_footprint (ground, z=0)
  └── base_link (chassis centre at z=0.15 m)
        ├── left_wheel_joint  (continuous) — y=+0.15
        ├── right_wheel_joint (continuous) — y=-0.15
        ├── caster_stem_joint (fixed)      — rear balance
        │     └── caster_joint (fixed)
        ├── imu_joint   (fixed)            — top of chassis
        └── laser_joint (fixed)            — top front of chassis
```

- Wheel track (centre-to-centre): **0.30 m** (`wheel_separation`)
- Wheel diameter: **0.18 m** (= 2 × 0.09 m radius)
- The caster ball has zero surface friction (`mu1=0`, `mu2=0`) to avoid interfering with differential drive dynamics.

#### Gazebo Plugins

| Plugin | Library | Output Topic |
|---|---|---|
| Differential Drive | `libgazebo_ros_diff_drive.so` | `/cmd_vel` (in), `/odom` (out) |
| IMU | `libgazebo_ros_imu_sensor.so` | `/imu` |
| LIDAR | `libgazebo_ros_ray_sensor.so` | `/scan` |

---

### 4.2 Custom Gazebo World

**File:** `src/robot_bringup/worlds/training_world.world`

A custom 10 × 10 m enclosed arena (`training_world`) with the following static obstacles:

| Model | Type | Position (x, y) | Size |
|---|---|---|---|
| `wall_1` | Thin wall | (2.0, 0.0) | 0.2 × 2.0 × 1.0 m — red |
| `box_obstacle_1` | Box | (−1.5, 1.5) | 0.5 × 0.5 × 0.5 m — green |
| `cylinder_obstacle_1` | Cylinder | (0.0, −2.0) | r=0.3, h=1.0 m — blue |
| `box_obstacle_2` | Box | (3.0, −1.5) | 0.6 × 0.6 × 0.5 m — yellow |
| Boundary walls (×4) | Walls | N/S/E/W at ±5.0 m | 10 m long, 1 m tall |

Physics is configured with ODE at a 1 kHz real-time update rate (`max_step_size=0.001`).

---

### 4.3 Sensor Configuration

#### LIDAR (`/scan` — `sensor_msgs/LaserScan`)

| Parameter | Value |
|---|---|
| Scan range | −180° to +180° (full 360°) |
| Samples | 360 rays |
| Min range | 0.12 m |
| Max range | 10.0 m |
| Update rate | 10 Hz |
| Noise | Gaussian, σ = 0.01 m |
| Frame | `laser_link` |

#### IMU (`/imu` — `sensor_msgs/Imu`)

| Parameter | Value |
|---|---|
| Update rate | 50 Hz |
| Frame | `imu_link` |
| Initial orientation as reference | false |

#### Wheel Odometry (`/odom` — `nav_msgs/Odometry`)

Published by the differential drive plugin. Also publishes the `odom → base_link` TF transform.

---

### 4.4 Build & Launch Instructions

#### Step 1 — Clone / extract the workspace

```bash
cd ~
# (workspace already at ~/ros2_ws from the zip)
cd ros2_ws
```

#### Step 2 — Install dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

#### Step 3 — Build

```bash
colcon build --symlink-install
source install/setup.bash
```

> **Tip:** Always `source install/setup.bash` in every new terminal before running any launch file.

#### Step 4 — Launch the simulation with RViz

```bash
ros2 launch robot_bringup gazebo.launch.py
```

This opens:
- **Gazebo** with the `training_world` and the robot spawned at the origin.
- **RViz** with `robot.rviz` pre-configured to display the robot model, LIDAR scan, and TF frames.

To launch Gazebo only (no RViz) — used internally by SLAM/Nav2 launches:

```bash
ros2 launch robot_bringup simulation.launch.py
```

#### Step 5 — Drive the robot manually (optional)

In a new terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use `i / , / j / l` keys to drive; press `k` to stop.

---

### 4.5 SLAM — Building the Map

SLAM Toolbox is used in **async mapping** mode. It subscribes to `/scan` and `/odom` and publishes a live `map` topic.

#### Launch SLAM

```bash
ros2 launch robot_navigation slam.launch.py
```

This starts Gazebo + SLAM Toolbox + RViz (with the `nav2.rviz` config that shows the live map).

#### Drive to build the map

In a second terminal, run `teleop_twist_keyboard` and drive the robot around the arena until the full map is built.

#### Save the map

Once satisfied with the map, in a third terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/src/robot_navigation/maps/map
```

This saves `map.yaml` and `map.pgm`. The committed versions of these files are already in `src/robot_navigation/maps/`.

**Map metadata (`map.yaml`):**

```yaml
image: map.pgm
mode: trinary
resolution: 0.05        # 5 cm per pixel
origin: [-4.94, -4.95, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

### 4.6 Nav2 — Autonomous Navigation

Nav2 is launched with the pre-built map for **localisation (AMCL) + path planning + obstacle avoidance**.

#### Launch Navigation

```bash
ros2 launch robot_navigation navigation.launch.py
```

This starts Gazebo + the full Nav2 stack (AMCL, BT Navigator, Global Planner, Local Planner, Costmaps) + RViz.

To use a custom map path:

```bash
ros2 launch robot_navigation navigation.launch.py map:=/full/path/to/your/map.yaml
```

#### Set an Initial Pose (in RViz)

1. In RViz, click **"2D Pose Estimate"** on the toolbar.
2. Click and drag on the map at the robot's actual starting position to set the initial pose for AMCL.

#### Send a Navigation Goal (in RViz)

1. Click **"Nav2 Goal"** (or "2D Nav Goal") on the toolbar.
2. Click and drag on the map to set the goal position and orientation.
3. The robot will plan a path and navigate autonomously, avoiding all obstacles using the costmap.

#### Nav2 Stack Summary

| Component | Role |
|---|---|
| AMCL | Particle filter localisation using `/scan` + `/odom` |
| Global Costmap | Inflated obstacle map for global path planning |
| Local Costmap | Real-time costmap for obstacle avoidance |
| NavFn / SmacPlanner | Global path planner |
| DWB Controller | Local trajectory controller (velocity commands → `/cmd_vel`) |
| BT Navigator | Behaviour Tree orchestrator for the navigation task |

---

## 5. Task 2 — Custom Messages, Services & Nodes

### 5.1 Custom Message — `Person.msg`

**Package:** `training_interfaces`
**File:** `src/training_interfaces/msg/Person.msg`

```
string name
int32 age
```

Generated C++ and Python bindings are produced automatically by `rosidl_generate_interfaces` at build time.

---

### 5.2 Custom Service — `Value.srv`

**Package:** `training_interfaces`
**File:** `src/training_interfaces/srv/Value.srv`

```
int32 a
int32 b
---
int32 val
```

The `---` separator divides the **request** fields (`a`, `b`) from the **response** field (`val`).

---

### 5.3 Publisher & Subscriber

**Package:** `training`

| Node | File | Topic | Type | Rate |
|---|---|---|---|---|
| `person_publisher` | `training/person_publisher.py` | `/person` | `training_interfaces/Person` | 1 Hz |
| `person_subscriber` | `training/person_subscriber.py` | `/person` | `training_interfaces/Person` | (event-driven) |

The publisher sends a fixed `Person` message (`name="Jimil"`, `age=21`) every second. The subscriber listens and logs each received message.

**Publisher excerpt:**

```python
msg = Person()
msg.name = 'Jimil'
msg.age = 21
self.publisher_.publish(msg)
```

**Subscriber excerpt:**

```python
def listener_callback(self, msg: Person):
    self.get_logger().info(f'Received: name={msg.name}, age={msg.age}')
```

---

### 5.4 Service Server & Client

**Package:** `training`

| Node | File | Service Name | Type |
|---|---|---|---|
| `service_server` | `training/service_server.py` | `add_two_ints` | `training_interfaces/Value` |
| `service_client` | `training/service_client.py` | `add_two_ints` | `training_interfaces/Value` |

The server receives `a` and `b`, computes `val = a + b`, and returns the response.

**Server logic:**

```python
def add_callback(self, request: Value.Request, response: Value.Response):
    response.val = request.a + request.b
    return response
```

The client accepts optional CLI arguments for `a` and `b` (defaults: `a=3`, `b=5`):

```bash
ros2 run training service_client 7 13   # → 7 + 13 = 20
```

---

### 5.5 Build & Run Instructions

#### Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### Run the Publisher/Subscriber pair

**Terminal 1 — Publisher:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run training person_publisher
```

**Terminal 2 — Subscriber:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run training person_subscriber
```

Expected subscriber output:

```
[INFO] [person_subscriber]: Received: name=Jimil, age=21
[INFO] [person_subscriber]: Received: name=Jimil, age=21
...
```

You can also echo the topic directly:

```bash
ros2 topic echo /person
```

#### Run the Service Server & Client

**Terminal 1 — Service Server:**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run training service_server
```

Expected output:

```
[INFO] [add_two_ints_server]: AddTwoIntsServer ready
```

**Terminal 2 — Service Client (default a=3, b=5):**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run training service_client
```

Expected output:

```
[INFO] [add_two_ints_client]: Result: 3 + 5 = 8
```

**With custom values:**

```bash
ros2 run training service_client 10 25
# [INFO] [add_two_ints_client]: Result: 10 + 25 = 35
```

You can also call the service directly from the CLI:

```bash
ros2 service call /add_two_ints training_interfaces/srv/Value "{a: 4, b: 6}"
```

---

## 6. ROS2 Topics Reference

### Task 1 — Simulation Topics

| Topic | Message Type | Publisher | Description |
|---|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo LIDAR plugin | 360° laser scan data |
| `/imu` | `sensor_msgs/Imu` | Gazebo IMU plugin | Orientation, angular velocity, linear acceleration |
| `/odom` | `nav_msgs/Odometry` | Diff-drive plugin | Wheel odometry (position + velocity) |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 / teleop | Velocity commands to the robot |
| `/robot_description` | `std_msgs/String` | `robot_state_publisher` | URDF XML string |
| `/tf` | `tf2_msgs/TFMessage` | RSP + plugins | Transform frames |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox / AMCL | Occupancy grid map |

### Task 2 — Training Topics

| Topic | Message Type | Publisher | Description |
|---|---|---|---|
| `/person` | `training_interfaces/Person` | `person_publisher` | Custom person data at 1 Hz |
| `/add_two_ints` | `training_interfaces/Value` | (service) | Addition service |

---

## 7. TF Frame Tree

```
map
 └── odom
      └── base_link
            ├── left_wheel
            ├── right_wheel
            ├── caster_stem
            │     └── caster_wheel
            ├── imu_link
            └── laser_link
```

- `map → odom`: published by AMCL (during Nav2) or SLAM Toolbox (during SLAM)
- `odom → base_link`: published by the `diff_drive` Gazebo plugin
- `base_link → *`: published by `robot_state_publisher` from the URDF

Visualise the TF tree at any time:

```bash
ros2 run tf2_tools view_frames
# Produces frames_<timestamp>.pdf and frames_<timestamp>.gv
```

A saved TF tree PDF is included in `Media_Proof/Task1_Nav2/Screenshots/TF Tree/`.

---

## 8. Media Proof Index

All proof of task completion is in `Media_Proof/`.

### Task 1 — Nav2 (`Media_Proof/Task1_Nav2/`)

#### Screen Recordings (`Screen Recording/`)

| File | Contents |
|---|---|
| `SLAM_Gazebo.webm` | SLAM running in Gazebo — robot driving and building the map |
| `SLAM_Mapping.webm` | SLAM Toolbox mapping in RViz |
| `Slam_map.webm` | Live map construction visible in RViz |
| `Map_saving.webm` | Terminal output of `map_saver_cli` saving the map |
| `RViz_Gazebo_RQT.webm` | RViz + Gazebo + RQT running simultaneously |
| `Topic_Echo_&_Map.webm` | `ros2 topic echo` showing live LIDAR/IMU/odom data + map |
| `Nav2_Proof.webm` | Autonomous goal navigation in Nav2 |
| `Nav_Vid.webm` | Additional Nav2 goal navigation clip |

#### Screenshots (`Screenshots/`)

| Folder | Contents |
|---|---|
| `Robot In Gazebo/` | Robot spawned in the custom `training_world` — multiple angles |
| `RViz/` | RViz showing robot model, LIDAR scan, and sensor data |
| `SLAM Map Building/` | RViz during active SLAM map construction |
| `Saved Maps/` | `map.yaml` + `map.pgm` — the saved occupancy grid |
| `Topic Echo/` | Terminal showing live topic data (`/scan`, `/imu`, `/odom`) |
| `TF Tree/` | `frames_*.pdf` + `frames_*.gv` — full TF tree |
| `NAV2 Navigation/` | Nav2 with goal set and robot navigating autonomously |

### Task 2 — ROS Programming (`Media_Proof/Task2_ROS_Programming/`)

| File | Contents |
|---|---|
| `Person_Pub_Sub.png` | Publisher and subscriber terminals showing message exchange |
| `Topic_Verification.png` | `ros2 topic list` and `ros2 topic echo /person` output |
| `Service_Server_Client.png` | Server and client terminals showing request/response |
| `Service_Verification.png` | `ros2 service list` and `ros2 service call` output |
| `Custom_msg+service.png` | `ros2 interface show` for both `Person.msg` and `Value.srv` |

### General

| File | Contents |
|---|---|
| `Folder_Structure.png` | Full workspace directory tree screenshot |

---

## 9. Troubleshooting

**Gazebo doesn't open / crashes immediately**

```bash
killall gzserver gzclient   # clean up orphan processes
ros2 launch robot_bringup gazebo.launch.py
```

**`xacro` errors on launch**

Ensure `xacro` is installed:

```bash
sudo apt install ros-humble-xacro
```

**`colcon build` fails on `training_interfaces`**

Make sure `rosidl_default_generators` is available:

```bash
sudo apt install ros-humble-rosidl-default-generators
```

**Nav2 robot doesn't move after setting a goal**

1. Confirm the initial pose is set with "2D Pose Estimate" in RViz first.
2. Check that `/scan` and `/odom` are publishing:
   ```bash
   ros2 topic hz /scan
   ros2 topic hz /odom
   ```
3. Verify Nav2 nodes are active:
   ```bash
   ros2 node list | grep nav
   ```

**Service client times out**

Make sure the service server is running **before** launching the client. The client will wait up to 1 second per attempt and retry.

**`source install/setup.bash` is always needed**

Add it to your `.bashrc` to avoid repeating it:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

*Workspace developed and tested on Ubuntu 22.04 LTS with ROS2 Humble Hawksbill and Gazebo Classic 11.*
