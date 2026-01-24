# ROS2 Robotic Arm Simulation (Gazebo + MoveIt2 + ros2_control)

## 📌 Project Introduction

This project implements a complete **ROS2 robotic arm simulation framework** based on a simplified Panda manipulator.

The system integrates:

- URDF/Xacro robot modeling
- ros2_control controllers
- Gazebo physics simulation
- MoveIt2 motion planning
- RViz visualization

It supports:

- RViz model visualization
- Gazebo physics simulation
- Joint position control
- Topic-based command control
- MoveIt motion planning & execution
- Modular ROS2 package design

---

## 🧠 System Architecture

```
MoveIt2 (planning)
      ↓
ros2_control (controllers)
      ↓
Gazebo (physics simulation)
      ↓
URDF Robot Model
```

---

## 📂 Workspace Structure

```
ros2_workspace/
└── src/
    ├── your_robot_description
    ├── your_robot_gazebo
    └── your_robot_moveit
```

Each package has a clear responsibility (standard industrial ROS design).

---

# 📦 Package Details

---

## 1️⃣ your_robot_description

Robot model + RViz visualization package

### Structure

```
your_robot_description
├── config
│   └── controllers.yaml
├── launch
│   ├── display.launch.py
│   └── rsp.launch.py
├── meshes
│   ├── collision
│   └── visual
└── urdf
    ├── simplified.urdf
    └── simplified.xacro
```

### Run

```bash
ros2 launch your_robot_description display.launch.py
```

---

## 2️⃣ your_robot_gazebo

Gazebo + ros2_control simulation backend

### Structure

```
your_robot_gazebo
├── config
│   ├── controllers.yaml
│   └── moveit_controllers.yaml
├── launch
│   ├── gazebo_launch.py
│   └── gazebo_moveit.launch.py
├── meshes
└── urdf
    ├── simplified.urdf
    ├── moveit.urdf
    └── gazebo_moveit.urdf
```

### Gazebo Only Mode

Start:

```bash
ros2 launch your_robot_gazebo gazebo_launch.py
```

Load controllers:

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_position_controller
```

Send command:

```bash
ros2 topic pub /arm_position_controller/commands \
std_msgs/msg/Float64MultiArray \
"{data: [0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.0]}"
```

---

### Gazebo + MoveIt Mode (recommended)

```bash
ros2 launch your_robot_gazebo gazebo_moveit.launch.py
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
```

Then use RViz MoveIt GUI to plan and execute trajectories.

---

## 3️⃣ your_robot_moveit

MoveIt2 planning configuration package

### Structure

```
your_robot_moveit
├── config
├── launch
├── meshes
└── urdf
```

### Run

```bash
ros2 launch your_robot_moveit demo.launch.py
```

---

# 🚀 Build Instructions

```bash
cd ~/ros2_workspace
colcon build
source install/setup.bash
```

---

# 🧪 Typical Workflow

```bash
colcon build
source install/setup.bash
ros2 launch your_robot_gazebo gazebo_moveit.launch.py
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
```

Plan motion in RViz.

---

# 🛠 Tech Stack

- ROS2 Humble
- Gazebo
- MoveIt2
- ros2_control
- URDF/Xacro
- RViz2

---

# ⭐ Highlights

- Modular package separation
- Gazebo + MoveIt integration
- ros2_control based architecture
- Ready for real hardware extension

---

# 👤 Author

Ricardo Guo

---

# 📄 License

MIT
