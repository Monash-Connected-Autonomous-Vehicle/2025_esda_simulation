# Lane Detection Feature

## Overview
The lane detection system uses computer vision to identify white lane markings from the robot's camera feed and publishes them as visual markers on the SLAM map.

## How to Use

### 1. Launch via UI (Recommended)
1. Run the UI launcher: `python3 src/esda_simulation_2025/scripts/ui_launch.py`
2. Build the workspace (button 1)
3. Launch the simulation (button 2)
4. Pick a detector mode from the Regular/FCN/TwinLiteNet+ dropdown
5. Click **"Launch Lane Detection"** (independent button, next to the dropdown -- click again to stop it)
6. Leave "Enable Lane Detection" checked if you also want SLAM/AMCL/Nav2 to consume `/scan_fused` instead of raw `/scan` when you launch those modules

Lane detection is a standalone module now, launched separately from SLAM (matching the other module buttons like
Launch SLAM/AMCL/Nav2). It shows a visualization window with 4 panels:
- **Top-left**: Camera feed with detected lane lines overlaid (+ drivable-area tint in TwinLiteNet+ mode)
- **Top-right**: Right camera feed (stereo reference)
- **Bottom-left**: Lane detection mask (white-pixel mask for Regular/FCN, model lane-segmentation mask for TwinLiteNet+)
- **Bottom-right**: Edge detection for Regular/FCN; the raw drivable-area segmentation mask for TwinLiteNet+

Lane markers are also published to `/lane_markers` (visible in RViz).

### 2. Manual Launch
If you prefer to launch manually:

```bash
# Source the workspace
source install/setup.bash

# Launch lane detection node
ros2 run esda_simulation_2025 lane_detection.py
```

## Visualizing in RViz2

To see the detected lanes in RViz:

1. Launch RViz2
2. Add a **MarkerArray** display:
   - Click "Add" button
   - Select "By topic"
   - Choose `/lane_markers`
3. The detected lanes will appear as yellow line segments in the camera frame

## Parameters

You can customize the lane detection behavior by passing parameters:

```bash
ros2 run esda_simulation_2025 lane_detection.py --ros-args \
  -p show_visualization:=true \
  -p white_threshold_low:=200 \
  -p white_threshold_high:=255 \
  -p min_line_length:=50 \
  -p max_line_gap:=50
```

### Parameter Descriptions:
- `show_visualization` (bool, default: true): Show the OpenCV visualization window
- `white_threshold_low` (int, default: 200): Lower threshold for white color detection (0-255)
- `white_threshold_high` (int, default: 255): Upper threshold for white color detection (0-255)
- `min_line_length` (int, default: 50): Minimum line length in pixels for Hough transform
- `max_line_gap` (int, default: 50): Maximum gap between line segments to connect them

## Algorithm Details

The lane detection uses a lightweight computer vision pipeline:

1. **Color Filtering**: Converts image to grayscale and applies threshold to isolate white pixels
2. **Edge Detection**: Uses Canny edge detector to find edges
3. **Region of Interest**: Focuses on lower 40% of image where ground appears
4. **Line Detection**: Uses Hough Line Transform to detect straight lines
5. **3D Projection**: Projects detected image lines to 3D world coordinates
6. **Publishing**: Publishes as MarkerArray for visualization in SLAM map

## Topics

### Subscribed Topics:
- `/camera_raw` (sensor_msgs/Image): Raw camera feed from simulation

### Published Topics:
- `/lane_markers` (visualization_msgs/MarkerArray): Detected lane lines as 3D markers

## Troubleshooting

### No lanes detected
- Check camera feed: `ros2 topic echo /camera_raw --once`
- Adjust white threshold parameters if ground texture is different
- Ensure there are white markings visible in the camera view

### Visualization window doesn't appear
- Set parameter: `show_visualization:=true`
- Ensure X11 forwarding is working (if using remote connection)

### Markers not visible in RViz
- Add MarkerArray display with topic `/lane_markers`
- Check frame: Markers are published in `camera_link` frame
- Ensure TF tree is complete

## Performance

The lane detection node is optimized for real-time performance:
- Processing rate: ~10-30 Hz (depending on image size and line complexity)
- CPU usage: Low (simple OpenCV operations)
- Memory: Minimal (<100MB)

## Alternative Detector: TwinLiteNet+

A third detector mode is available alongside Regular (classic CV) and FCN: **TwinLiteNet+**, a lightweight multi-task
(drivable-area + lane-line) segmentation model from [TwinLiteNetPlus](https://github.com/chequanghuy/TwinLiteNetPlus).
It plugs in the same way FCN does — `scripts/lane_detection_twinlite.py` subclasses `LaneDetectionNode` and only
overrides mask/line extraction and visualization. Everything downstream (3D projection, `/lane_markers`,
`/lane_obstacles`, and the `/scan_fused` SLAM injection in `scan_callback`) is the unmodified base-class code, so
lanes are tracked into the map exactly the same way as the other two modes.

### One-time setup

Unlike the FCN `.h5` file, a PyTorch checkpoint alone isn't enough — PyTorch needs the matching model class definition
to load a `state_dict`. So this mode needs the upstream repo's source *and* a pretrained weight file, both kept
outside this ROS package (same convention as `lane-detection-on-rural-roads-master` for FCN) and already listed in
`.gitignore`:

1. Repo (already done for this workspace, re-run if missing):
   ```bash
   cd ~/esda_simulation_2025
   git clone https://github.com/chequanghuy/TwinLiteNetPlus.git
   ```
2. Pretrained weights (`nano.pth`/`small.pth`/`medium.pth`/`large.pth`) — only distributed via the
   [Google Drive link in the TwinLiteNetPlus README](https://github.com/chequanghuy/TwinLiteNetPlus#pre-trained-model),
   download manually and place under `TwinLiteNetPlus/pretrained/`, e.g. `TwinLiteNetPlus/pretrained/nano.pth`.
3. PyTorch must be importable by the node's Python interpreter (`python3 -c "import torch"`). It's already installed
   in this environment as the CPU-only `python3-torch` apt package — no GPU acceleration even though a GPU may be
   present (see performance note below).

### Selecting it

In the UI (`ui_launch.py`), pick **TwinLiteNet+** in the Regular/FCN/TwinLiteNet+ dropdown, then click
**"Launch Lane Detection"**. It defaults to the `nano` variant. To run manually:

```bash
ros2 run esda_simulation_2025 lane_detection_twinlite.py --ros-args \
  -p twinlite_repo_path:=$HOME/esda_simulation_2025/TwinLiteNetPlus \
  -p twinlite_weight_path:=$HOME/esda_simulation_2025/TwinLiteNetPlus/pretrained/nano.pth \
  -p twinlite_variant:=nano
```

### Parameters

- `twinlite_repo_path` (default `TwinLiteNetPlus`): path to the cloned repo (resolved relative to cwd, script dir, or workspace root)
- `twinlite_weight_path` (default `TwinLiteNetPlus/pretrained/nano.pth`): path to the `.pth` checkpoint
- `twinlite_variant` (default `nano`): must match the checkpoint — `nano`/`small`/`medium`/`large`
- `twinlite_device` (default `auto`): `auto` picks `cuda` if `torch.cuda.is_available()`, else `cpu`; falls back to CPU with a warning if `cuda` is requested but unavailable
- `twinlite_img_size` (default `640`): letterbox target size, matches the repo's own preprocessing exactly
- `twinlite_lane_confidence` (default `0.5`): probability threshold on the lane-line class after softmax
- `twinlite_temporal_window` (default `5`): number of frames averaged (on the raw probability field) for stable tracking
- `twinlite_draw_area` (default `true`): blend the model's drivable-area mask into the debug visualization only (not published as a topic)
- `twinlite_overlay_alpha` (default `0.45`): blend strength for the drivable-area overlay
- `twinlite_max_inference_hz` (default `0.0` = uncapped): optional throttle if CPU inference starts lagging `/scan_fused`

### Performance note

Model variants trade speed for accuracy (34K params/nano up to 1.94M params/large). On CPU-only PyTorch, nano
benchmarks at roughly 100ms/frame (~8-9 Hz) on this workstation — noticeably slower than the classic detector's
10-30 Hz. To keep the SLAM-critical `/scan_fused` path responsive despite this, the node runs its image callback on
a separate `ReentrantCallbackGroup` under a `MultiThreadedExecutor`, so a slow inference pass never blocks
`scan_callback`'s TF lookup and republish. If it's still not enough, drop to a smaller variant, set
`twinlite_max_inference_hz`, or run on a machine with a CUDA-enabled PyTorch build.

## Future Enhancements

Possible improvements:
- Lane following controller (use detected lanes for autonomous navigation)
- Lane departure warning
- Curved lane detection (currently only straight lines)
- Machine learning-based detection for better accuracy
- Integration with Nav2 costmap as obstacles
