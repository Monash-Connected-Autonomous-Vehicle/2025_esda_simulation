# How to Launch the ZED2 Camera

## Quick Command Reference

### Method 1: Using the GUI (Easiest)

1. Start the Real Robot Manager GUI:
```bash
cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025
python3 src/esda_simulation_2025/scripts/ui_launch_real_robot.py
```

2. Click the green **"Launch ZED2 Camera"** button in the "Hardware Integration" section

3. A new xterm window will open showing the camera initialization and startup messages

4. The camera will publish to `/hardware/zed/*` namespace - verify with:
```bash
ros2 topic list | grep hardware/zed
```

5. To stop the camera, click the button again (it will turn red when running)

---

### Method 2: Command Line (Manual)

Run this command in your terminal:

```bash
cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025
source install/setup.bash
export CUDA_HOME=/usr/local/cuda-12.2
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH
ros2 launch esda_hardware_2025 zed_hardware.launch.py
```

---

## Testing the Camera

### Test 1: View Available Topics

```bash
# List all ZED camera topics
ros2 topic list | grep hardware/zed

# Should show ~113 topics including:
# /hardware/zed/left/image_rect_color
# /hardware/zed/right/image_rect_color
# /hardware/zed/depth/depth_registered
# /hardware/zed/imu/data
# /hardware/zed/odom
# /hardware/zed/zed/temperature
```

### Test 2: View Camera Feed in RViz2

```bash
# Launch RViz2
rviz2

# Add Image Display:
# 1. Click "Add" button
# 2. Select "Image" under "rviz_default_plugins"
# 3. Set Topic to: /hardware/zed/left/image_rect_color
# 4. You should see the live camera feed
```

### Test 3: Test with ZED SDK Tool

```bash
export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH
/usr/local/zed/tools/ZED_Depth_Viewer

# This opens a standalone ZED viewer showing:
# - Left and right RGB images
# - Real-time depth map
# - Performance metrics
# Press ESC to exit
```

### Test 4: Echo Topic Data

```bash
# View image metadata
ros2 topic echo /hardware/zed/left/camera_info --once

# View IMU data
ros2 topic echo /hardware/zed/imu/data --once

# View odometry
ros2 topic echo /hardware/zed/odom --once
```

---

## Troubleshooting

### Camera Not Detected
```bash
# Verify camera is connected
lsusb | grep -i zed

# Should show: ID 2b03:f880 STEREOLABS ZED 2i

# Check USB 3.0 connection
lsusb -t | grep -i stereolabs
# Should show 5000M or 10000M bandwidth, NOT 480M
```

### Topics Not Publishing
```bash
# Check if node is running
ros2 node list | grep hardware

# Check for errors
ros2 node info /hardware/zed/zed

# View node logs
ros2 topic echo /rosout | grep hardware
```

### CUDA Errors
```bash
# Verify CUDA installation
nvcc --version

# Verify GPU is available
nvidia-smi

# Rebuild if needed
cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025
colcon build --packages-select esda_hardware_2025
```

---

## Published Topics Reference

| Topic | Type | Description |
|-------|------|-------------|
| `/hardware/zed/left/image_rect_color` | sensor_msgs/Image | Left camera RGB |
| `/hardware/zed/right/image_rect_color` | sensor_msgs/Image | Right camera RGB |
| `/hardware/zed/depth/depth_registered` | sensor_msgs/Image | Depth map (16-bit) |
| `/hardware/zed/point_cloud/cloud_registered` | sensor_msgs/PointCloud2 | 3D point cloud |
| `/hardware/zed/imu/data` | sensor_msgs/Imu | Inertial measurement data |
| `/hardware/zed/odom` | nav_msgs/Odometry | Camera odometry |
| `/hardware/zed/zed/temperature` | sensor_msgs/Temperature | Camera temperature |
| `/hardware/zed/left/camera_info` | sensor_msgs/CameraInfo | Left camera calibration |
| `/hardware/zed/right/camera_info` | sensor_msgs/CameraInfo | Right camera calibration |

---

## Configuration

Camera parameters are in `/tmp/zed_params_override.yaml`:

```yaml
# Resolution levels:
# 0=Native, 1=2K (2208x1242), 2=HD1080 (1920x1080)
# 3=HD720 (1280x720), 4=VGA (672x376)
resolution: 4

# Frame rate in Hz (default: 10 for USB bandwidth optimization)
pub_frame_rate: 10.0

# Depth processing mode: 0=PERFORMANCE, 1=BALANCED, 2=QUALITY
depth_mode: 0
```

To change settings, edit the file and restart the camera.

---

## Integration Notes

- Camera publishes to `/hardware/zed/*` namespace (not `/zed/*`)
- This prevents conflicts with simulation nodes
- Can run simultaneously with Gazebo simulation
- IMU, odometry, and depth data all available for navigation algorithms
