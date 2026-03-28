# Autonomous_Warehouse_Inventory_Scanning_Robot
A differential drive robot capable of detecting racks in a warehouse environment, autonomously navigates to them and scan the contents by detecting and decoding the qr code placed on each shelf.

---


https://github.com/user-attachments/assets/6b3d509c-c802-48ed-a41d-49db0b997484


---

## System Architecture

The system follows a modular ROS 2 pipeline covering mapping, localization, perception, navigation, control, and visualisation.

### 1. Mapping and Localization
- The `ros2_laser_scan_matcher` package provides LiDAR-based odometry using scan matching (ICP).
- The `ekf_node` fuses wheel odometry, IMU data, and LiDAR odometry into a robust filtered odometry, publishes transform `odom → base_footprint` for local estimation.
- The `slam_toolbox` package performs real-time 2D SLAM to generate an occupancy grid map of the environment and continuously corrects the odometry drift, publishes transform `map → odom` for global localization.

### 2. Rack Detection
- The `rack_detector` node processes the occupancy grid map to identify rack candidates based on geometric features and visualizes the detections using Matplotlib.
- Detected racks are published to a topic using the custom `warehouse_msgs` interfaces (`Rack` and `RackArray`).

### 3. Mission Control
- The `warehouse_mission_control` package coordinates the overall autonomous workflow.
- The `mission_executor` node manages high-level task execution by sending navigation goals via Nav2 and controlling the camera joint for vertical scanning.
- It controls the transitions between navigation and perception states based on mission logic.
  
### 4. QR Code Detection
- The `qr_pipeline` node processes the camera stream to detect QR codes using a YOLO-based model (`models/qr_model.pt`).
- The detected region is then preprocessed and decoded to extract the shelf content information.
  
### 5. Control
- The robot is controlled using `ros2_control`, with the `gz_ros2_control` plugin interfacing the controllers with the Gazebo simulation.
- A `diff_drive_controller` based controller is used for planar (x–y and yaw) motion of the robot base, a `joint_trajectory_controller` based camera joint controller is used for vertical camera movement while a `joint_state_broadcaster` based controller publishes joint states for feedback and visualization.
  
### 6. Visualisation
- RViz visualizes the map, robot pose, planned path, and live QR detection feed.

---

## Repository Structure

```text
src/
├── warehouse_robot_bringup/                # System bringup (launch + configs)
│   ├── launch/
│   │   └── final.launch.py                 # Full system launch (Gazebo + SLAM + Nav2 + controllers + Rviz)
│   ├── config/
│   │   ├── nav2_params.yaml               # Nav2 configuration 
│   │   ├── ekf.yaml                       # EKF localization parameters
│   │   ├── mapper_params_online_async.yaml # SLAM Toolbox parameters
│   │   ├── ros2_controller.yaml           # ros2_control controllers
│   │   └── rviz_config.rviz               # RViz visualization config
│
├── warehouse_robot_description/             # Robot model + simulation assets
│   ├── urdf/
│   │   ├── warehouse_robot.urdf           # Robot URDF model
│   │   └── warehouse_robot.sdf            # Robot SDF model
│   ├── worlds/
│   │   ├── warehouse.sdf                  # Warehouse simulation world
│   │   ├── meshes/                        # Rack and environment meshes
│   │   └── textures/                      # QR textures for racks
│
├── warehouse_mission_control/               # High-level mission execution
│   ├── warehouse_mission_control/
│   │   ├── mission_executor.py            # Main mission node (navigation + scanning pipeline)
│   │   ├── qr_pipeline.py                 # QR detection + pre-processing + decoding
│   ├── models/
│   │   └── qr_model.pt                    # Trained QR detection YOLO model
│
├── warehouse_rack_detection/               # LiDAR-based Rack detection 
│   ├── warehouse_rack_detection/
│   │   ├── rack_detector.py              # Main detection node (LaserScan → racks)
│
├── warehouse_msgs/                       # Custom ROS2 message definitions
│   ├── msg/
```
---

## Installation

```bash
# Create workspace
mkdir -p ~/warehouse_ws/src
cd ~/warehouse_ws/src

# Clone repository
git clone https://github.com/Anany444/Autonomous_Warehouse_Inventory_Scanning_Robot.git

# Move to workspace root
cd ~/warehouse_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install
source install/setup.bash
```

### Python Dependencies
```bash
pip install ultralytics opencv-python zxing-cpp matplotlib
```
---

## Usage
### 1. Launch the Full System
This launch starts the Gazebo simulation, robot state publisher, ekf, SLAM,  Nav2, the rack detector, the QR pipeline, the mission executor, and RViz.

```bash
# Source the workspace
source ~/warehouse_ws/install/setup.bash

#Launch everything
ros2 launch warehouse_robot_bringup final.launch.py
```

### 2. Mission Management
The mission executor uses `/racks_found` and `/start_navigation`  services to manage tasks during runtime.

1. **Map the environment and detect racks**  
   Use teleoperation to explore the warehouse and build the map. Ensure that racks are detected in the `rack_detector` Matplotlib window:

  ```bash
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
2. **Store detected rack locations**
Once all racks are detected, call the `/racks_found` service to store their center coordinates:

```bash
ros2 service call /racks_found std_srvs/srv/Trigger
```
3. **Start Autonomous mission**
Move the robot to the warehouse entry point and call the `/start_navigation` service to begin mission execution:

```bash
ros2 service call /start_navigation std_srvs/srv/Trigger
```
4. **Monitor QR decoding output**
The robot navigates to each rack and scans the QR codes. The decoded data can be monitored via:

```bash
# In a new terminal, source the workspace
source ~/warehouse_ws/install/setup.bash

# Echo decoded qr output string
ros2 topic echo /qr_model/output_string
```
---
