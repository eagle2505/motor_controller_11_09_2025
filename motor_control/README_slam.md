# 🦾 Motor Control Package: SLAM (Mapping) & AMCL (Localization and Autonomous movement)

This README provides instructions for setting up and running SLAM (Simultaneous Localization and Mapping).

---

Overview

SLAM: Used to create a map of an unknown environment while simultaneously keeping track of the robot's location within it.

---

SLAM Mapping

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


### 3. 🔧 Build the ROS 2 Package

Open a terminal in your ROS2 in your source folder (not motor control package) (e.g. `~/roamio_brain/`) and run:

```bash
colcon build
```

---

### 4. 🪄 Source the Environment

After building the workspace, source the setup script:
```bash
. install/setup.bash
```

---

### 5. 🧠 Run the Launch File

To launch the SLAM Mapping, run:
```bash
ros2 launch motor_control slam.launch.py 
```

If everything is launched correctly, you will be able to see the robot's TF, its lidar data and mapped surroundings.

---

### 7. ⌨️ Controls

To move the robot around use the teleop_keyboard window and use the keys mentioned in this window.

[I] - Move Forward
[,] - Move Backward
[J] - Rotate Left
[L] - Rotate Right
[K] - Stop the Robot
[Q] - Increase Speed
[Z] - Decrease Speed

You have to move the robot around your locations thoroughly to create a proper map.

---

### 8.  Save your map

```bash
ros2 run nav2_map_server map_saver_cli -f <map_name>
```

This will generate:

- <map_name>.yaml
- <map_name>.pgm (or .png)


Save the map in the specific folder:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/roamio42/Desktop/<name of the parent folder>/motor_control/maps
```

