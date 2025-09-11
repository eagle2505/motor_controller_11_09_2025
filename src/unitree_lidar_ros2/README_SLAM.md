# Using Unitree 4D LiDAR L2 with SLAM Toolbox

This setup allows you to use the Unitree 4D LiDAR L2 with SLAM Toolbox by converting the 3D point cloud data to 2D laser scan data.

## How It Works

1. **Unitree 4D LiDAR Node** (`unitree_lidar_ros2_node`): Publishes 3D point clouds on `/unilidar/cloud` topic
2. **Point Cloud to Laser Scan Converter** (`pointcloud_to_laserscan_node`): Converts 3D point clouds to 2D laser scans and publishes on `/scan` topic
3. **SLAM Toolbox**: Receives the 2D laser scan data on `/scan` topic and performs SLAM mapping

## Usage

### 1. Build the Package
```bash
cd ~/Desktop/aaqil/lidar
colcon build --packages-select unitree_lidar_ros2
source install/setup.bash
```

### 2. Launch the LiDAR and Converter
```bash
ros2 launch unitree_lidar_ros2 lidar_converter_launch.py
```

This will:
- Start the Unitree 4D LiDAR node
- Start the point cloud to laser scan converter
- Publish 2D laser scan data on `/scan` topic

### 3. Launch SLAM Toolbox (in a new terminal)
```bash
ros2 launch slam_toolbox online_async_launch.py
```

## Configuration

The converter node has several parameters you can adjust:

- **`min_height` / `max_height`**: Height range to include in the 2D scan (default: -0.5 to 0.5 meters)
- **`angle_min` / `angle_max`**: Angular range of the scan (default: -π to π radians)
- **`angle_increment`**: Angular resolution (default: 0.0174533 radians = 1 degree)
- **`range_min` / `range_max`**: Distance range limits (default: 0.1 to 50.0 meters)
- **`target_frame`**: Coordinate frame for the output scan (default: "base_link")

## Topics

- **Input**: `/unilidar/cloud` (PointCloud2 from 4D LiDAR)
- **Output**: `/scan` (LaserScan for SLAM Toolbox)
- **IMU**: `/unilidar/imu` (IMU data from 4D LiDAR)

## Coordinate Frames

- `unilidar_lidar`: LiDAR sensor frame
- `unilidar_imu`: IMU sensor frame  
- `base_link`: Robot base frame (where the 2D scan is published)

## Troubleshooting

1. **No scan data**: Check if the LiDAR is publishing on `/unilidar/cloud`
2. **TF errors**: Ensure proper coordinate frame transformations
3. **Poor scan quality**: Adjust `min_height`/`max_height` parameters for your environment

## Notes

- The converter filters points by height to create a 2D slice of the 3D environment
- Only the closest point at each angle is used (like a traditional 2D LiDAR)
- The 4D LiDAR's additional data (intensity, multiple returns) is not used for 2D SLAM 