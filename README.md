#  Differential Drive Mobile Robot with Intel RealSense D455


A complete ROS2 Humble workspace featuring a 6-wheel differential drive mobile robot equipped with an Intel RealSense D455 camera for simulation and real-world deployment.

---


##  Features

###  Differential Drive Robot
- **6-wheel skid-steering configuration** for enhanced traction
- **Robust differential drive controller** with odometry feedback
- **Customizable robot dimensions** via URDF/xacro files
- **Realistic physics simulation** in Gazebo

###  Intel RealSense D455 Camera Integration
- **RGB Camera**: 1280×720 @ 30Hz
- **Depth Camera**: 1280×720 @ 30Hz
- **3D Point Cloud**: Real-time generation for perception tasks
- **Hardware-accurate specifications**: FOV, range, physical dimensions

###  Simulation & Visualization
- **Gazebo Classic**: Full physics-based simulation
- **RViz2**: Real-time 3D visualization
- **TF Tree**: Complete transform hierarchy
- **Sensor Plugins**: Camera, depth, point cloud generation

---

##  Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (Jammy)
- **ROS2**: Humble Hawksbill
- **Gazebo**: Gazebo Classic 11
- **Python**: 3.10+

### Required ROS2 Packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-rviz2 \
  ros-humble-tf2-tools \
  ros-humble-realsense2-camera \
  python3-colcon-common-extensions
```

---

##  Installation

### 1. Clone the Repository
This command creates a new folder called `ws_ddmobile` in your home directory and clones the repository into it:
```bash
cd ~
git clone https://github.com/1sznkaveesh/D455-Camera-Integration-ROS2.git ws_ddmobile
cd ws_ddmobile
```

### 2. Install Dependencies
```bash
cd ~/ws_ddmobile
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the Workspace
```bash
colcon build --symlink-install
```

### 4. Source the Workspace
```bash
source install/setup.bash
```

**Tip**: Add this to your `~/.bashrc` for automatic sourcing:
```bash
echo "source ~/ws_ddmobile/install/setup.bash" >> ~/.bashrc
```

---

##  Quick Start

### Launch the Robot in Gazebo
```bash
source ~/ws_ddmobile/install/setup.bash
ros2 launch mobile_dd_robot gazebo_model.launch.py
```

This will:
-  Start Gazebo with an empty world
-  Spawn the robot with the D455 camera
-  Initialize all sensors and controllers
-  Publish transforms and robot state

### Visualize in RViz2
Open a new terminal:
```bash
source ~/ws_ddmobile/install/setup.bash
rviz2
```

**Quick RViz2 Setup:**
1. **Fixed Frame**: Set to `odom`
2. **Add**: RobotModel
3. **Add**: Image → Topic: `/camera/d455/image_raw`
4. **Add**: PointCloud2 → Topic: `/camera/d455/points`
   - Size (m): `0.1`
   - Style: `Flat Squares`
   - Color Transformer: `RGB8`

---

##  Usage

### Control the Robot

#### Using Teleop Twist Keyboard (Optional)
```bash
sudo apt install ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### View Camera Feeds

#### RGB Image
```bash
ros2 run rqt_image_view rqt_image_view /camera/d455/image_raw
```

#### Depth Image
```bash
ros2 run rqt_image_view rqt_image_view /camera/d455/depth/image_raw
```

### Monitor Topics
```bash
# List all active topics
ros2 topic list

# Echo odometry data
ros2 topic echo /odom

# Check camera info
ros2 topic echo /camera/d455/camera_info
```

### Visualize TF Tree
```bash
# View TF tree
ros2 run tf2_tools view_frames

# Open the generated PDF
evince frames.pdf
```

---

### Workspace Structure
```
ws_ddmobile/
├── build/              # Build artifacts (auto-generated)
├── install/            # Installation directory (auto-generated)
├── log/                # Build and runtime logs (auto-generated)
├── src/
│   └── mobile_dd_robot/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── README.md
│       ├── launch/
│       │   └── gazebo_model.launch.py
│       └── model/
│           ├── camera.xacro         # D455 camera URDF
│           ├── robot.gazebo         # Gazebo plugins
│           └── robot_with_camera.xacro  # Complete robot URDF
└── README.md           # This file
```

##  Topics & Services

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Velocity commands for robot control |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/msg/Odometry` | Robot odometry (position, velocity) |
| `/camera/d455/image_raw` | `sensor_msgs/msg/Image` | RGB camera feed |
| `/camera/d455/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB camera calibration |
| `/camera/d455/depth/image_raw` | `sensor_msgs/msg/Image` | Depth image |
| `/camera/d455/depth/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth camera calibration |
| `/camera/d455/points` | `sensor_msgs/msg/PointCloud2` | 3D point cloud |
| `/tf` | `tf2_msgs/msg/TFMessage` | Transform tree |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | Static transforms |

### Transform (TF) Tree
```
odom
 └── body_link
      ├── left_wheel_1_link
      ├── left_wheel_2_link
      ├── left_wheel_3_link
      ├── right_wheel_1_link
      ├── right_wheel_2_link
      ├── right_wheel_3_link
      └── camera_link
           ├── camera_depth_frame
           └── camera_optical_frame
```
---
<div align="center">
</div>
