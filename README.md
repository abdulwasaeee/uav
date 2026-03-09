# UAV — VLM-Based Autonomous Drone System

A fully autonomous drone simulation built on ROS2 Humble + Gazebo. The drone can see, remember, and navigate to objects using AI — all running locally with no cloud required.

---

## What This System Does

Imagine a drone that:
- Flies around a room and **remembers every object it sees** ("red cone at position X", "person near the wall")
- Can be told in plain English: **"go to the fire hydrant you saw earlier"** and actually flies there
- **Avoids obstacles automatically** — if a wall is in the way it goes around it
- Can **map an entire room by itself** without you controlling it
- Builds a **3D map** of the environment as it flies

This is all done using a combination of AI vision (YOLO), 3D mapping (RTAB-Map), depth sensors, and a semantic memory system (ChromaDB).

---

## System Architecture

```
Camera Feed
    │
    ▼
YOLO (detects objects at 2Hz)
    │
    ▼
LLaVA (describes objects: "orange cone near left wall")
    │
    ▼
ChromaDB (saves label + description + 3D position to disk)

You type: "go to the orange cone"
    │
    ▼
Memory search → finds cone position
    │
    ▼
Controller flies drone there at 0.4 m/s
    │
    ▼
Obstacle Mux (intercepts if wall detected) → safe navigation
```

---

## What Each Node Does

### `tf_relay.py` — The Translator
ROS2 has strict rules about coordinate frame names. The drone simulator uses names with a `/` at the start (like `/simple_drone/base_link`) which ROS2 rejects. This node fixes all those names silently in the background. **Must always be the first node you start.**

### `tof_to_laserscan.py` — The Wall Radar
The drone has 6 depth sensors pointing in different directions (front, front-left, front-right, rear, rear-left, rear-right). Each sensor tells us how far away the nearest object is in that direction. This node combines all 6 into a single laser scan that RTAB-Map uses to detect walls.

### `obstacle_mux_node.py` — The Safety Guard
This is the most important safety node. It sits between the AI brain and the actual drone motors. At 50 times per second it checks all 6 depth sensors. If anything is within 0.8 meters — it overrides whatever the AI is telling the drone to do and moves away from the obstacle. Nothing can bypass this. Think of it as an automatic emergency brake.

### `yolo_detector_node.py` — The Eyes
Uses YOLOv8 (a fast object detection AI) to scan the camera feed 2 times per second. For every object it finds (person, chair, bottle, cone, etc.) it calculates the exact 3D position of that object in the world using the depth camera. Publishes detections as JSON.

### `semantic_map_node.py` — The Memory Writer
Receives detections from YOLO. For each new object it:
1. Asks LLaVA to write a human description ("orange traffic cone near the left wall")
2. Saves the label, description, and 3D position to ChromaDB on disk
3. Shows floating labels in RViz so you can see what the drone knows

The database persists between sessions — the drone remembers objects from previous flights.

### `memory_query_node.py` — The Memory Reader (optional, replaced by VLM node)
A simple CLI to search the semantic memory by typing object descriptions. Searches by matching words against stored labels and descriptions.

### `vlm_node.py` — The Brain / Main Control Interface
The main node you interact with. Type commands in plain English:
- `takeoff` / `land` / `stop`
- `go to the person near the dumpster`
- `find the fire hydrant`
- `list` — show everything the drone remembers
- `status` — current state

Parses your command using keyword matching, searches the semantic memory, and sends the goal position to the controller. The controller then flies the drone there while the obstacle mux keeps it safe.

### `controller_node.py` — The Pilot (alternative to VLM node)
A standalone position controller with its own CLI. Receives goal positions and flies the drone there using a proportional controller at 20Hz. Can be used instead of the VLM node for direct control without natural language.

### `exploration_node.py` — The Explorer
Tells the drone to map the entire room autonomously. Type `start` and it:
1. Records its current position as home
2. Generates an expanding spiral pattern of waypoints
3. Flies to each waypoint at 0.3 m/s (slow for clean mapping)
4. Uses ToF sensors to avoid walls automatically
5. When all waypoints are done — flies back to start and lands

RTAB-Map builds the 3D map as the drone flies.

---

## Hardware This Was Designed For

| Component | Spec |
|---|---|
| Drone frame | DJI F450 |
| Compute | NVIDIA Jetson Orin Nano (8GB) |
| Front camera | RGB-D depth camera (640×480, 30Hz) |
| Depth sensors | 6× A010 ToF sensors (70°×60° FOV, 0.2–2.5m range) |
| Bottom sensor | A010 ToF facing downward (altitude hold) |
| OS | Ubuntu 22.04 + ROS2 Humble |

---

## Prerequisites

- Ubuntu 22.04
- Docker installed
- Git installed
- Ollama installed on host machine

### Install Ollama
```bash
curl -fsSL https://ollama.com/install.sh | sh
ollama pull llava
ollama pull nomic-embed-text
```

---

## Setup

### 1. Clone the repo
```bash
git clone https://github.com/abdulwasaeee/UAV.git
cd UAV
```

### 2. Build the base Docker image (one time only, ~15 minutes)
```bash
cd drone
bash build_base.sh
```

### 3. Build the dev image (~3 minutes, run after any code changes)
```bash
bash run_docker.sh
```

This automatically rebuilds if needed and launches Gazebo.

---

## Running the Full System

Open a new terminal for each step. Run them in this exact order.

### Terminal 1 — Simulation
```bash
cd ~/UAV/drone
bash run_docker.sh
```
Wait for Gazebo to fully open before starting the next terminals.

### Terminal 2 — TF Relay (always first)
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && python3 /nodes/tf_relay.py"
```

### Terminal 3 — ToF Laser Scan
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && python3 /nodes/tof_to_laserscan.py"
```

### Terminal 4 — RTAB-Map (3D mapping)
```bash
docker exec -it sjtu_drone bash -c "
source /opt/ros/humble/setup.bash &&
ros2 launch rtabmap_launch rtabmap.launch.py \
  frame_id:=simple_drone/base_footprint \
  odom_frame_id:=simple_drone/odom \
  map_frame_id:=map \
  rgb_topic:=/simple_drone/rgbd/image_raw_fixed \
  depth_topic:=/simple_drone/rgbd/depth/image_raw_fixed \
  camera_info_topic:=/simple_drone/rgbd/camera_info_fixed \
  odom_topic:=/simple_drone/odom \
  scan_topic:=/simple_drone/scan \
  visual_odometry:=false \
  approx_sync:=true \
  approx_sync_max_interval:=0.5 \
  queue_size:=30 \
  rviz:=false \
  wait_for_transform:=0.5 \
  delete_db_on_start:=true \
  Rtabmap/DetectionRate:=2 \
  Rtabmap/TimeThr:=0 \
  Grid/CellSize:=0.05 \
  Grid/RangeMax:=10.0 \
  Grid/RayTracing:=true \
  Grid/FromDepth:=false \
  RGBD/LinearUpdate:=0.1 \
  RGBD/AngularUpdate:=0.05 \
  Mem/STMSize:=30
"
```

### Terminal 5 — Obstacle Mux (safety)
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /nodes/obstacle_mux_node.py"
```

### Terminal 6 — YOLO Object Detection
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /nodes/yolo_detector_node.py"
```

### Terminal 7 — Semantic Map
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /nodes/semantic_map_node.py"
```

### Terminal 8 — VLM Control (main interface)
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /nodes/vlm_node.py"
```

---

## Using the System

### Basic flight
```
vlm> takeoff
vlm> stop
vlm> land
```

### Navigate to a remembered object
```
vlm> takeoff
vlm> list                          # see what drone knows
vlm> go to the person              # fly to a person
vlm> go to fire hydrant            # fly to fire hydrant
vlm> find the orange cone          # search and fly
```

### Autonomous room mapping
In Terminal 8 (VLM): `vlm> takeoff`

Open Terminal 9:
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && python3 /nodes/exploration_node.py"
```
```
explore> start     # drone maps entire room and returns home automatically
explore> stop      # stop at any time
explore> status    # check progress
```

---

## Viewing the Map

In RViz (opens automatically with Gazebo) add these displays:
- `/rtabmap/cloud_map` → PointCloud2 (3D colored map)
- `/rtabmap/map` → OccupancyGrid (2D floor plan)
- `/simple_drone/scan` → LaserScan (ToF sensor rays)
- `/semantic_map/markers` → MarkerArray (floating object labels)
- `/detections/image` → Image (live YOLO detections)

---

## Saving the Map

Before closing the simulation:
```bash
docker cp sjtu_drone:/root/.ros/rtabmap.db ~/UAV/vlm/maps/rtabmap.db
```

To view a saved map later:
```bash
docker run -it --rm \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=${DISPLAY} \
  -v ${HOME}/UAV/vlm/maps:/root/.ros \
  sjtu_drone_local:humble \
  bash -c "source /opt/ros/humble/setup.bash && rtabmap-databaseViewer /root/.ros/rtabmap.db"
```

---

## Project Structure

```
UAV/
├── drone/                          # Docker + ROS2 simulation
│   ├── Dockerfile                  # Base image (build once)
│   ├── Dockerfile.dev              # Dev image (fast rebuild)
│   ├── run_docker.sh               # Launch script
│   ├── build_base.sh               # Base image build script
│   ├── sjtu_drone_description/     # Drone URDF, meshes, plugins
│   │   ├── urdf/sjtu_drone.urdf.xacro   # F450 drone with 6 ToF sensors
│   │   └── models/sjtu_drone/     # 3D mesh files
│   ├── sjtu_drone_bringup/         # Launch files and world config
│   └── sjtu_drone_control/         # Low-level flight control
│
└── vlm/
    └── nodes/                      # All AI nodes
        ├── tf_relay.py             # Fix ROS2 TF frame names
        ├── tof_to_laserscan.py     # 6 ToF → LaserScan
        ├── obstacle_mux_node.py    # Safety — obstacle avoidance override
        ├── yolo_detector_node.py   # YOLO object detection + 3D position
        ├── semantic_map_node.py    # Save objects to ChromaDB
        ├── memory_query_node.py    # Search object memory
        ├── vlm_node.py             # Main control interface
        ├── controller_node.py      # Position controller
        ├── exploration_node.py     # Autonomous room mapping
        └── sjtu_drone_ov/          # OpenVINS config (IMU+camera odometry)
            ├── estimator_config.yaml
            ├── kalibr_imu_chain.yaml
            └── kalibr_imucam_chain.yaml
```

---

## AI Models Used

| Model | Purpose | Size | Speed |
|---|---|---|---|
| YOLOv8n | Object detection | 6MB | 2Hz (CPU) |
| LLaVA | Object description | 4.7GB | ~1 per new object |
| nomic-embed-text | Text embeddings | 270MB | Fast |
| ChromaDB | Vector database | — | Instant |

All models run locally. No internet required after initial download.

---

## Troubleshooting

**Drone not moving after `takeoff`**
Make sure obstacle_mux_node is running. The drone only receives commands through the mux.

**YOLO not detecting anything**
Check that tf_relay is running and `/simple_drone/rgbd/image_raw_fixed` is publishing:
```bash
docker exec -it sjtu_drone bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /simple_drone/rgbd/image_raw_fixed"
```

**Map not building / disappearing**
Fly slowly. RTAB-Map needs overlapping frames to build keyframes. Max recommended speed is 0.5 m/s.

**Memory empty after restart**
The semantic map database is saved at `/root/.ros/semantic_map_db` inside the container. Add a volume mount to persist it:
```bash
# In run_docker.sh add:
-v ${HOME}/UAV/vlm/maps:/root/.ros \
```

**`chromadb` or `ultralytics` not found**
These are installed in the dev Docker image. Make sure you built with `bash run_docker.sh` not the base image directly.

---

## Contributing

Pull requests welcome. Key areas for improvement:
- OpenVINS integration for better odometry (config in `vlm/nodes/sjtu_drone_ov/`)
- Phi-3 Vision to replace LLaVA (faster, more accurate descriptions)
- TensorRT optimization for Jetson deployment
- Autonomous exploration improvement with better frontier detection
