# ESDA Hardware 2025 - ZED2 Camera Integration# ESDA Hardware 2025 - ZED2 Camera Integration



This package integrates the Stereolabs ZED2 camera into the ESDA simulation stack with proper ROS2 topic namespacing to avoid collisions with simulation nodes.This package integrates the Stereolabs ZED2 camera into the ESDA simulation stack with proper ROS2 topic namespacing to avoid collisions with simulation nodes.



## Hardware Setup## Hardware Setup



### Camera Specifications### Camera Specifications

- **Model**: Stereolabs ZED 2 / ZED 2i- **Model**: Stereolabs ZED 2 / ZED 2i

- **USB Requirement**: USB 3.0 port (minimum 10 Gbps / 5000M bandwidth recommended)- **USB Requirement**: USB 3.0 port (minimum 10 Gbps / 5000M bandwidth recommended)

- **Resolution**: 400p (672x376) with 10 Hz framerate (optimized for USB bandwidth)- **Resolution**: 400p (672x376) with 10 Hz framerate (optimized for USB bandwidth)

- **Depth Mode**: PERFORMANCE- **Depth Mode**: PERFORMANCE

- **ROS2 Namespace**: `/hardware/zed/*` (isolated from simulation topics)- **ROS2 Namespace**: `/hardware/zed/*` (isolated from simulation topics)



### Physical Connection### Physical Connection

1. Connect ZED camera to a **USB 3.0 port** (blue ports, NOT USB 2.0 black ports)1. Connect ZED camera to a **USB 3.0 port** (blue ports, NOT USB 2.0 black ports)

2. Ensure camera is fully seated in the USB port2. Ensure camera is fully seated in the USB port

3. Verify connection: `lsusb | grep -i zed` should show device3. Verify connection: `lsusb | grep -i zed` should show device



## Environment Setup## Environment Setup



### Prerequisites### Prerequisites

- ROS 2 Humble (or compatible distribution)- ROS 2 Humble (or compatible distribution)

- CUDA 12.2 toolkit installed at `/usr/local/cuda-12.2`- CUDA 12.2 toolkit installed at `/usr/local/cuda-12.2`

- NVIDIA driver 595.58.03 or compatible- NVIDIA driver 595.58.03 or compatible

- ZED SDK 4.2.5 installed at `/usr/local/zed`- ZED SDK 4.2.5 installed at `/usr/local/zed`



### CUDA Environment Variables### CUDA Environment Variables



The launch script automatically sets these, but you may need them for manual testing:The launch script automatically sets these, but you may need them for manual testing:



```bash```bash

export CUDA_HOME=/usr/local/cuda-12.2export CUDA_HOME=/usr/local/cuda-12.2

export PATH=$CUDA_HOME/bin:$PATHexport PATH=$CUDA_HOME/bin:$PATH

export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATHexport LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

``````



## Running the ZED Camera Node## Running the ZED Camera Node



### Quick Start### Quick Start



```bash```bash

cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025

source install/setup.bashsource install/setup.bash

export CUDA_HOME=/usr/local/cuda-12.2export CUDA_HOME=/usr/local/cuda-12.2

export PATH=$CUDA_HOME/bin:$PATHexport PATH=$CUDA_HOME/bin:$PATH

export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATHexport LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

ros2 launch esda_hardware_2025 zed_hardware.launch.pyros2 launch esda_hardware_2025 zed_hardware.launch.py

``````



### With Manual Setup Script### With Manual Setup Script



A shell script is provided for convenience:A shell script is provided for convenience:



```bash```bash

bash ./launch_zed_camera.shbash ./launch_zed_camera.sh

``````



## Testing the ZED Camera## Testing the ZED Camera



### Test 1: ZED SDK Test Tool### Test 1: ZED SDK Test Tool



Test if the ZED SDK can detect and initialize the camera:Test if the ZED SDK can detect and initialize the camera:



```bash```bash

export LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATHexport LD_LIBRARY_PATH=/usr/local/cuda-12.2/lib64:$LD_LIBRARY_PATH

/usr/local/zed/tools/ZED_Depth_Viewer/usr/local/zed/tools/ZED_Depth_Viewer

``````



This will open a GUI showing real-time depth and RGB streams from the camera. Press `ESC` to exit.This will open a GUI showing real-time depth and RGB streams from the camera. Press `ESC` to exit.



### Test 2: Check ROS2 Topics### Test 2: Check ROS2 Topics



Verify the camera is publishing to ROS2 topics:Verify the camera is publishing to ROS2 topics:



```bash```bash

# List all camera topics# List all camera topics

ros2 topic list | grep hardware/zedros2 topic list | grep hardware/zed



# Example output:# Example output:

# /hardware/zed/camera_info# /hardware/zed/camera_info

# /hardware/zed/depth/camera_info# /hardware/zed/depth/camera_info

# /hardware/zed/depth/image_raw# /hardware/zed/depth/image_raw

# /hardware/zed/imu/data# /hardware/zed/imu/data

# /hardware/zed/left/camera_info# /hardware/zed/left/camera_info

# /hardware/zed/left/image_rect_color# /hardware/zed/left/image_rect_color

# /hardware/zed/odom# /hardware/zed/odom

# /hardware/zed/point_cloud/cloud_registered# /hardware/zed/point_cloud/cloud_registered

# /hardware/zed/right/camera_info# /hardware/zed/right/camera_info

# /hardware/zed/right/image_rect_color# /hardware/zed/right/image_rect_color

# /hardware/zed/zed/odometry# /hardware/zed/zed/odometry

# /hardware/zed/zed/path# /hardware/zed/zed/path

``````



### Test 3: Visualize in RViz2### Test 3: Visualize in RViz2



Launch RViz2 and visualize camera data:Launch RViz2 and visualize camera data:



```bash```bash

rviz2rviz2

``````



Then add image displays for:Then add image displays for:

- `/hardware/zed/left/image_rect_color` (left camera feed)- `/hardware/zed/left/image_rect_color` (left camera feed)

- `/hardware/zed/depth/image_raw` (depth map)- `/hardware/zed/depth/image_raw` (depth map)

- `/hardware/zed/point_cloud/cloud_registered` (3D point cloud)- `/hardware/zed/point_cloud/cloud_registered` (3D point cloud)



### Test 4: Echo Topics### Test 4: Echo Topics



Display live data from a specific topic:Display live data from a specific topic:



```bash```bash

# View left camera images# View left camera images

ros2 topic echo /hardware/zed/left/image_rect_colorros2 topic echo /hardware/zed/left/image_rect_color



# View IMU data# View IMU data

ros2 topic echo /hardware/zed/imu/dataros2 topic echo /hardware/zed/imu/data



# View odometry# View odometry

ros2 topic echo /hardware/zed/odomros2 topic echo /hardware/zed/odom

``````



## Published Topics## Published Topics



All topics are published under the `/hardware/zed/` namespace:All topics are published under the `/hardware/zed/` namespace:



### Image Topics### Image Topics

- `/hardware/zed/left/image_rect_color` - Left camera RGB image- `/hardware/zed/left/image_rect_color` - Left camera RGB image

- `/hardware/zed/right/image_rect_color` - Right camera RGB image- `/hardware/zed/right/image_rect_color` - Right camera RGB image

- `/hardware/zed/depth/image_raw` - Depth map (16-bit)- `/hardware/zed/depth/image_raw` - Depth map (16-bit)

- `/hardware/zed/rgb/image_rect_color` - RGB image- `/hardware/zed/rgb/image_rect_color` - RGB image



### Camera Info Topics### Camera Info Topics

- `/hardware/zed/left/camera_info` - Left camera calibration- `/hardware/zed/left/camera_info` - Left camera calibration

- `/hardware/zed/right/camera_info` - Right camera calibration- `/hardware/zed/right/camera_info` - Right camera calibration

- `/hardware/zed/depth/camera_info` - Depth camera calibration- `/hardware/zed/depth/camera_info` - Depth camera calibration



### Point Cloud### Point Cloud

- `/hardware/zed/point_cloud/cloud_registered` - 3D point cloud in camera frame- `/hardware/zed/point_cloud/cloud_registered` - 3D point cloud in camera frame



### Odometry & Pose### Odometry & Pose

- `/hardware/zed/odom` - Odometry (camera motion estimation)- `/hardware/zed/odom` - Odometry (camera motion estimation)

- `/hardware/zed/pose` - Camera pose- `/hardware/zed/pose` - Camera pose

- `/hardware/zed/zed/odometry` - Alternative odometry format- `/hardware/zed/zed/odometry` - Alternative odometry format

- `/hardware/zed/zed/path` - Historical path taken by camera- `/hardware/zed/zed/path` - Historical path taken by camera



### IMU & Sensors### IMU & Sensors

- `/hardware/zed/imu/data` - Inertial Measurement Unit data- `/hardware/zed/imu/data` - Inertial Measurement Unit data

- `/hardware/zed/zed/temperature` - Camera temperature- `/hardware/zed/zed/temperature` - Camera temperature



## Camera Parameters## Camera Parameters



Parameters can be modified in `/tmp/zed_params_override.yaml`:Parameters can be modified in `/tmp/zed_params_override.yaml`:



```yaml```yaml

# Resolution: 0=Native (Depends on camera model)# Resolution: 0=Native (Depends on camera model)

#             1=2K (2208x1242)#             1=2K (2208x1242)

#             2=HD1080 (1920x1080) #             2=HD1080 (1920x1080) 

#             3=HD720 (1280x720)#             3=HD720 (1280x720)

#             4=VGA (672x376)#             4=VGA (672x376)

resolution: 4resolution: 4



# Publish frame rate in Hz (10 Hz for USB bandwidth optimization)# Publish frame rate in Hz (10 Hz for USB bandwidth optimization)

pub_frame_rate: 10.0pub_frame_rate: 10.0



# Depth mode: 0=PERFORMANCE, 1=BALANCED, 2=QUALITY# Depth mode: 0=PERFORMANCE, 1=BALANCED, 2=QUALITY

depth_mode: 0depth_mode: 0

``````



To change parameters, edit the file and relaunch the camera node.To change parameters, edit the file and relaunch the camera node.



## Troubleshooting## Troubleshooting



### Camera Not Detected### Camera Not Detected



**Error**: `CAMERA NOT DETECTED` or `Error opening camera`**Error**: `CAMERA NOT DETECTED` or `Error opening camera`



**Solutions**:**Solutions**:

1. Verify camera is connected: `lsusb | grep -i zed`1. Verify camera is connected: `lsusb | grep -i zed`

2. Check USB 3.0 port: `lsusb -t | grep Stereolabs` should show 5000M or 10000M bandwidth2. Check USB 3.0 port: `lsusb -t | grep Stereolabs` should show 5000M or 10000M bandwidth

3. Try different USB 3.0 ports on the back of the PC3. Try different USB 3.0 ports on the back of the PC

4. Restart the camera driver:4. Restart the camera driver:

   ```bash   ```bash

   pkill -9 -f "ros2 launch"   pkill -9 -f "ros2 launch"

   sleep 2   sleep 2

   # Then relaunch   # Then relaunch

   ```   ```



### LOW USB BANDWIDTH Error### LOW USB BANDWIDTH Error



**Error**: `Unable to capture images. Consider trying a lower resolution and/or FPS`**Error**: `Unable to capture images. Consider trying a lower resolution and/or FPS`



**Solutions**:**Solutions**:

1. Ensure camera is on USB 3.0 port (blue), not USB 2.01. Ensure camera is on USB 3.0 port (blue), not USB 2.0

2. Try a different USB port2. Try a different USB port

3. Check for damaged USB cable3. Check for damaged USB cable

4. The launch file is already optimized for low bandwidth (400p @ 10Hz)4. The launch file is already optimized for low bandwidth (400p @ 10Hz)

5. Consider external powered USB 3.0 hub if issue persists5. Consider external powered USB 3.0 hub if issue persists



### SDK Not Found### SDK Not Found



**Error**: `ZED SDK Version: Not detected` or missing library errors**Error**: `ZED SDK Version: Not detected` or missing library errors



**Solutions**:**Solutions**:

1. Verify SDK installation: `ls -la /usr/local/zed/`1. Verify SDK installation: `ls -la /usr/local/zed/`

2. Check library path: `echo $LD_LIBRARY_PATH` should include `/usr/local/cuda-12.2/lib64`2. Check library path: `echo $LD_LIBRARY_PATH` should include `/usr/local/cuda-12.2/lib64`

3. Reinstall ZED SDK if needed3. Reinstall ZED SDK if needed



### CUDA/GPU Issues### CUDA/GPU Issues



**Error**: `CUDA initialization failed` or GPU-related errors**Error**: `CUDA initialization failed` or GPU-related errors



**Solutions**:**Solutions**:

1. Verify NVIDIA driver: `nvidia-smi`1. Verify NVIDIA driver: `nvidia-smi`

2. Check CUDA installation: `nvcc --version`2. Check CUDA installation: `nvcc --version`

3. Ensure CUDA environment is set correctly3. Ensure CUDA environment is set correctly

4. Check GPU availability: `nvidia-smi` should list GPU device4. Check GPU availability: `nvidia-smi` should list GPU device



## Configuration Files## Configuration Files



### Launch File### Launch File

- **Location**: `launch/zed_hardware.launch.py`- **Location**: `launch/zed_hardware.launch.py`

- **Purpose**: Main launch entry point- **Purpose**: Main launch entry point

- **Usage**: `ros2 launch esda_hardware_2025 zed_hardware.launch.py`- **Usage**: `ros2 launch esda_hardware_2025 zed_hardware.launch.py`



### Parameters Override### Parameters Override

- **Location**: `/tmp/zed_params_override.yaml`- **Location**: `/tmp/zed_params_override.yaml`

- **Purpose**: Custom camera parameters (resolution, framerate, depth mode)- **Purpose**: Custom camera parameters (resolution, framerate, depth mode)

- **Auto-generated** by launch file if not present- **Auto-generated** by launch file if not present



## Integration with Simulation## Integration with Simulation



The camera topics are isolated under `/hardware/zed/` namespace to prevent conflicts with simulation nodes. This allows:The camera topics are isolated under `/hardware/zed/` namespace to prevent conflicts with simulation nodes. This allows:



- Running simulation and real camera simultaneously- Running simulation and real camera simultaneously

- Easy topic remapping if needed- Easy topic remapping if needed

- Clear separation of hardware vs simulation data- Clear separation of hardware vs simulation data



To use camera data in your nodes, subscribe to topics like:To use camera data in your nodes, subscribe to topics like:

```python```python

# Example ROS2 Python node# Example ROS2 Python node

def image_callback(msg):def image_callback(msg):

    # Process camera image    # Process camera image

    pass    pass



self.subscription = self.create_subscription(self.subscription = self.create_subscription(

    Image,    Image,

    '/hardware/zed/left/image_rect_color',    '/hardware/zed/left/image_rect_color',

    image_callback,    image_callback,

    10    10

))

``````



## Building from Source## Building from Source



```bash```bash

cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025cd ~/Desktop/esda_sim_2025_repo/esda_simulation_2025

colcon build --packages-select esda_hardware_2025colcon build --packages-select esda_hardware_2025

``````



## Additional Resources## Additional Resources



- [ZED ROS2 Wrapper Documentation](https://github.com/stereolabs/zed-ros2-wrapper)- [ZED ROS2 Wrapper Documentation](https://github.com/stereolabs/zed-ros2-wrapper)

- [ZED SDK Documentation](https://www.stereolabs.com/docs/overview/)- [ZED SDK Documentation](https://www.stereolabs.com/docs/overview/)

- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)



## Contact & Support## Contact & Support



For issues or questions about the hardware integration, refer to the main repository documentation or open an issue in the project tracker.For issues or questions about the hardware integration, refer to the main repository documentation or open an issue in the project tracker.

