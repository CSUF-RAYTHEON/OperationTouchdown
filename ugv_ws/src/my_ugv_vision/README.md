# my_ugv_vision

BrainChip Akida AKD1000 perception pipeline for the UGV. Runs a quantized YOLO
detector on the AKD1000, fuses detections with depth from the OAK-D and ranges
from the RPLiDAR (merged scan), tracks unique world-frame landmarks, and emits
both visualization markers and Nav2 obstacle points.

## Nodes

| Node | Purpose | Outputs |
|------|---------|---------|
| `akida_yolo_node` | Runs YOLO inference on AKD1000 | `/akida/detections`, `/akida/image_annotated`, `/akida/inference_latency_ms`, `/diagnostics` |
| `object_localizer_node` | Lifts 2D detections to 3D using fused depth | `/akida/landmarks_raw` |
| `landmark_tracker_node` | Deduplicates landmarks in `map` frame | `/akida/markers`, `/akida/landmark_array` |
| `obstacle_publisher_node` | Publishes landmarks as a PointCloud2 for Nav2 | `/akida/obstacle_points` |

## Prerequisites

1. ROS 2 Jazzy on Ubuntu 24.04, with `vision_msgs`, `cv_bridge`, `image_geometry`,
   `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `foxglove_bridge`,
   `nav2_costmap_2d` installed (covered by `rosdep install --from-paths src`).
2. **AKD1000 PCIe driver and userspace must be installed.** The `akida` and
   `akida-models` Python packages are NOT available via apt or rosdep and must
   be installed with pip into the same Python interpreter ROS 2 Jazzy uses
   (system Python 3.12 on Ubuntu 24.04):

   ```bash
   pip install --break-system-packages akida akida-models
   ```

   (Or use a venv linked into your overlay.)
3. Verify the AKD1000 is visible:

   ```bash
   akida devices
   ```

   You should see one entry like `device 0: PCIe/AKD1000`. If empty, the node
   will fall back to a virtual device and log a prominent WARN; perception still
   runs but at much lower throughput.

## Build & run

```bash
cd /home/meow/Desktop/OperationTouchdown/ugv_ws
colcon build --packages-select my_ugv_vision
source install/setup.bash
ros2 launch my_ugv_vision perception.launch.py
```

Or, for the full stack (UGV bringup + perception):

```bash
ros2 launch my_ugv_bringup master.launch5.py
```

## Foxglove layout

Open Foxglove Studio, hit *Layouts → Import from file…*, and pick:

```
ugv_ws/src/my_ugv_bringup/foxglove_layouts/ugv_perception.json
```

It contains 3D view, annotated image, raw RGB, inference latency plot, topic
graph, and diagnostics panels pre-wired to the perception topics.

## Model assets

The deployable model and its metadata live in `models/`:

- `models/ugv_yolo_akida.fbz` — Akida-converted YOLO (deploy to AKD1000).
- `models/model_metadata.json` — labels, anchors, input shape, grid size, alpha.

Originals are kept in the repo's top-level `akida/` directory along with the
training H5 artifacts. Do not delete those — they are required to retrain.
