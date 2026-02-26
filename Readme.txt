# SiMMER (Legged Mobile Manipulator) - Project Documentation

SiMMER is a ROS (catkin) project for a **6-legged mobile base with a manipulator**.  
This repository contains:

- `lmm_model`: robot description, meshes, controllers, and simulation launch files
- `lmm_sc`: supervisory control stack for real-time planning, joystick input handling, gait planning, redundancy resolution, and hardware servo output

Originally documented as:
- Version: 1.1
- Date: 26 February 2020
- Core ideas: Adaptive Stroke Length, Full Cycle Planning, optimization-based solving (SciPy `fsolve`)

---

## Repository Structure

```
SiMMER/
├── Readme.txt
├── lmm_model/
│   ├── urdf/lmm.urdf
│   ├── meshes/*.STL
│   ├── config/controllers.yaml
│   ├── launch/{display.launch,gazebo.launch}
│   └── src/joint_controller.py
└── lmm_sc/
    ├── launch/{start.launch,simulation.launch}
    ├── config/joints.yaml
    └── src/
        ├── master.py
        ├── joy_incremental.py
        ├── motor_controller.py
        ├── tip_tf_broadcaster.py
        └── modules/*.py
```

---

## Requirements

### System
- Ubuntu with ROS 1 (catkin-based setup)
- Python (ROS `rospy` scripts)

### ROS packages used in this repository
- `catkin`
- `roscpp`
- `rospy`
- `sensor_msgs`
- `std_msgs`
- `robot_state_publisher`
- `joint_state_publisher`
- `rviz`
- `gazebo` / `gazebo_ros`
- `controller_manager`
- `joy`
- `i2cpwm_board` (required for hardware motor output path)

### Python modules used by control nodes
- `numpy`
- `scipy` (for optimization-based solver paths)
- `rospkg`
- `yaml`
- ROS TF libraries (`tf2_ros`, TF messages)

---

## Setup (Catkin Workspace)

1. Create a catkin workspace (if you do not already have one):
   - `mkdir -p ~/catkin_ws/src`
   - `cd ~/catkin_ws/src`
2. Place this repository in `~/catkin_ws/src/SiMMER`.
3. Build:
   - `cd ~/catkin_ws`
   - `catkin_make`
4. Source:
   - `source devel/setup.bash`

> Note: In this repository snapshot, `catkin_make` availability depends on your ROS installation and shell environment.

---

## Running the Project

### 1) Visualization (URDF + RViz)
Run:
- `roslaunch lmm_model display.launch`

This starts:
- `joint_state_publisher`
- `robot_state_publisher`
- `rviz` with `lmm_model/rviz/lmm.rviz`

### 2) Gazebo Simulation + Supervisory Control
Run:
- `roslaunch lmm_sc simulation.launch`

This launches Gazebo, spawns the model, loads controllers, and starts:
- joystick input node
- TF tip broadcaster
- supervisory controller (`master.py`)
- simulation joint command bridge (`lmm_model/src/joint_controller.py`)

### 3) Hardware-oriented supervisory control path
Run:
- `roslaunch lmm_sc start.launch`

This runs:
- joystick input
- supervisory controller
- motor controller (`motor_controller.py`) publishing `ServoArray` to `servos_absolute`
- `i2cpwm_board` node for PWM/servo output

---

## Main Nodes and Responsibilities

### `lmm_sc/src/master.py`
Central real-time supervisory control node:
- subscribes to incremental end-effector commands (`lmm_incremental_inputs`)
- performs redundancy handling and inverse kinematics
- plans trunk-body and leg-tip trajectories when base motion is required
- publishes complete robot joint states (`lmm_joint_states`)
- logs runtime data to `lmm_sc/data/<timestamp>.csv`

### `lmm_sc/src/joy_incremental.py`
- subscribes to `joy` (`sensor_msgs/Joy`)
- maps joystick axes to Cartesian incremental commands
- publishes `lmm_incremental_inputs` (`Float32MultiArray`)

### `lmm_sc/src/motor_controller.py`
- subscribes to `lmm_joint_states`
- maps joint angles to servo board input values using `lmm_sc/config/joints.yaml`
- publishes `servos_absolute` (`i2cpwm_board/ServoArray`)

### `lmm_model/src/joint_controller.py`
- simulation bridge node
- subscribes to `lmm_joint_states`
- republishes each joint command to Gazebo controllers under `/lmm/*_position_controller/command`

### `lmm_sc/src/tip_tf_broadcaster.py`
- publishes tip transforms for all six legs and the end-effector tip on `/tf`

---

## Core Topics

- `joy` (`sensor_msgs/Joy`) -> joystick input
- `lmm_incremental_inputs` (`std_msgs/Float32MultiArray`) -> Cartesian increments
- `lmm_joint_states` (`sensor_msgs/JointState`) -> planned joint commands/state
- `/lmm/.../command` (`std_msgs/Float64`) -> Gazebo controller commands
- `servos_absolute` (`i2cpwm_board/ServoArray`) -> hardware servo outputs

---

## Key Configuration Files

- `lmm_sc/src/modules/inputs.py`
  - kinematic dimensions
  - initial trunk/end-effector pose
  - timing (`f_in`, `T_in`, `T_stroke`)
  - manipulability thresholds and optimization bounds

- `lmm_sc/config/joints.yaml`
  - per-joint servo calibration:
    - servo channel
    - min/max joint angles
    - min/max board input values
    - axis direction multiplier

- `lmm_model/config/controllers.yaml`
  - Gazebo effort position controllers and PID gains

---

## Data Logging

`master.py` automatically logs runtime data rows to:
- `lmm_sc/data/<YYYY_MM_DD__HH_MM_SS>.csv`

Logged values include time, input increments, end-effector and trunk position terms, manipulability indicator, stroke length, and selected joint angles.

---

## Notes

- This project is ROS 1/catkin oriented.
- Some package metadata is still template/default (for example, `lmm_sc/package.xml` license field is `TODO`).
- The repository contains both simulation and hardware pathways; choose launch files accordingly.
