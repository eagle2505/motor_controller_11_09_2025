# 🦾 Motor Control Package: AMCL (Localization and Autonomous movement)

This README provides instructions for setting up AMCL (Adaptive Monte Carlo Localization) for your robot using ROS2.

---

Overview

AMCL: Used for localization of the robot on a known map (pre-built with SLAM) using particle filters and autonomous movement, using Rviz and pin-point navigation.

---

AMCL

### 1. 🔌 Upload the Arduino code

Upload the following script to your Arduino using the Arduino IDE or `arduino-cli`:

- `./arduino_scripts/nav2_motor_control/nav2_motor_control.ino`:
    enables an Arduino to receive velocity commands from ROS 2, control the robot's motors and send back odometry data to pi.

Make sure your Arduino is connected and appears on `/dev/ttyACM0` or adjust the port as needed in the ROS 2 node's parameters.

If there are any problems with the code uploading and you are using Pi 5, check if the necessary permissions are opened for the Arduino port.

- `ls -la /dev/ttyACM0` (for the port ACM0)

Change the permissons if needed:

- `sudo 777 /dev/ttyACM0` (for the port ACM0)

---

### 2. 🔌 Check the Lidar permissions

Check if the necessary permissions are opened for the Lidar port.

- `ls -la /dev/ttyUSB0` (for the port USB0)

Change the permissons if needed:

- `sudo 777 /dev/ttyUSB0` (for the port USB0)

By default Lidar should be connected to the USB0 port. If it's not this exact port and you are using Pi5, the order of plugging in the devices to the USB ports should be the following:

- Arduino (so it takes ACM0)
- Lidar (so it takes USB0)
- any other device

---

### 3. Upload the map you want to use for navigation

All of the maps are currently in the motor_control package /maps folder. By default this package uses 19.06_16.11.yaml file (map of the auditorium).

If you want to use another map, change it's name in `localization.launch.py` file at default_map_path variable. The map should be .yaml file (pre-built and saved using SLAM). As default it use the path <current parent folder>/motor_control/maps

---

### 4. 🔧 Build the ROS 2 Package

Open a terminal in your ROS2 in your source folder (not motor control package) (e.g. `~/roamio_brain/`) and run:

```bash
colcon build
```

---

### 5. 🪄 Source the Environment

After building the workspace, source the setup script:
```bash
. install/setup.bash
```

---

### 6. 🧠 Run the Launch File

To launch the SLAM Mapping, run:
```bash
ros2 launch motor_control localization.launch.py 
```

After the launch if you have following issues here are ways to fix them:

- Map: not visible and has a warning "map not received"
    - check that Map-> Topic-> Durability Policy is set to "Transient Local". If not, change it.
    - double check the path for the map that you are providing

- TF: the robot's URDF is not apperinng
    - make sure that the script that you're uploaded to the Arduino is nav2_motor_control.ino
---

### 7. Set the start and end position

In the Rviz window on the upper panel click 2D Pose Estimate and click the approximate robot's position on the map. 

Click the 2D Goal Pose to set the end position. Click and drag the mouse to set the destination + the vector.

Your robot moved successfully. Congrats!
