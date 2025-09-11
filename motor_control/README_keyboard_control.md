# 🦾 Motor Control Package: Keyboard control (ROS 2)

This package provides keyboard-based teleoperation for controlling a four-motor robot using an Arduino and ROS 2. It supports both basic speed control and PID-based motor speed regulation.

---

## 📦 Features

- Keyboard control of a four-motor robot
- Two Arduino firmware modes:
  - Direct speed control
  - PID-based control for smoother motion
- Written in Python (no external libraries required)
- Clean interface for ROS 2 beginners

---

## 🚀 Getting Started

### 1. 🔌 Upload the Arduino Code

Upload one of the following scripts to your Arduino using the Arduino IDE or `arduino-cli`:
its located under arduino_scripts folder.

- `keyboard_control.ino`:  
  Basic motor control via speed commands.
  
- `motor_control_with_PID.ino`:  
  Advanced control using PID for more accurate speed handling.

Make sure your Arduino is connected and appears on `/dev/ttyACM0` or adjust the port as needed in the ROS 2 node's parameters.

---

### 2. 🔧 Build the ROS 2 Package

Open a terminal in your ROS2 motor control package (e.g. `~/roamio_brain/motor_control`) and run:

```bash
colcon build
```

### 3. 🪄 Source the Environment

After building the workspace, source the setup script:
```bash
. install/setup.bash
```

### 4. 🧠 Run the Node

To launch the motor control node, run:
```bash
ros2 run motor_control keyboard_control
```


### 5. ⌨️ Controls

Use the following keys to control the robot:

[W] - Move Forward
[S] - Move Backward
[A] - Crab Left
[D] - Crab Right
[ctrl + c] - stop the program




