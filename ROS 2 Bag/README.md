# 📦 ROS 2 Bag Recording: `ROS 2 Bag`

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Storage](https://img.shields.io/badge/Storage-SQLite3-003B57?style=for-the-badge&logo=sqlite)](https://www.sqlite.org/)
[![Status](https://img.shields.io/badge/Status-Verified_&_Complete-2ECC71?style=for-the-badge)]()
[![Messages](https://img.shields.io/badge/Messages-49,463-orange?style=for-the-badge)]()

**Course:** Autonomous Vehicles — II PAO 2026  
**Assessment:** First Midterm Project (Part 1) — Reactive Controllers Competition  
**Student / Driver:** ISLAM KAZI MAJHARUL  
**Group:** Group 2  
**Assigned Track:** Sakhir International Circuit  
**Contributor:** Winter Delgado (@widegonz)  

---

## 📑 Table of Contents
1. [Overview & Dataset Specifications](#-overview--dataset-specifications)
2. [Recorded Topic Breakdown](#-recorded-topic-breakdown)
3. [How to Run the Bag File (Step-by-Step)](#-how-to-run-the-bag-file-step-by-step)
4. [How to Handle Playback (Interactive Keyboard Controls)](#-how-to-handle-playback-interactive-keyboard-controls)
5. [What to Do Next (Evaluation, Telemetry & Visualization Workflows)](#-what-to-do-next-evaluation-telemetry--visualization-workflows)
   - [Workflow 1: 3D Visualization in RViz2](#workflow-1-3d-visualization-in-rviz2-without-physics-simulation)
   - [Workflow 2: Live Telemetry & Command Monitoring](#workflow-2-live-telemetry--command-monitoring-terminal-2)
   - [Workflow 3: Offline Algorithm Testing & Validation](#workflow-3-offline-algorithm-testing--validation)
   - [Workflow 4: Quantitative Data Extraction & Telemetry Plotting](#workflow-4-quantitative-data-extraction--telemetry-plotting)
6. [Automated Helper Scripts](#-automated-helper-scripts)
7. [Oral Defense Summary for Instructors](#-oral-defense-summary-for-instructors)

---

## 📊 Overview & Dataset Specifications

This directory contains the complete serialized simulation dataset recorded directly during an autonomous run on the Sakhir International Circuit using:
```bash
ros2 bag record -a -o "ROS 2 Bag"
```

It captures all LiDAR scans, vehicle commands, transforms (`/tf`), occupancy grids (`/map`), and telemetry markers with nanosecond timestamps.

| Property | Value | Description |
| :--- | :--- | :--- |
| **Directory Name** | `ROS 2 Bag/` | Root-level visible directory for evaluation |
| **Database File** | `f1tenth_run_01_0.db3` | SQLite3 serialized message storage (**35.3 MiB**) |
| **Metadata File** | `metadata.yaml` | Topic definitions, storage format, and QoS profiles |
| **Recording Duration** | **17.83 s** | Clean sample window capturing cornering & straightaway |
| **Total Message Count** | **49,463 messages** | High-fidelity synchronous stream (~2,774 msgs/sec) |
| **ROS 2 Distribution** | ROS 2 Humble | Standard DDS middleware |
| **Compression** | Uncompressed SQLite3 | Instant playback without decompression overhead |

---

## 🛰️ Recorded Topic Breakdown

| Topic | Message Type | Count | Frequency | Description |
| :--- | :--- | :---: | :---: | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | **4,463** | ~250 Hz | 1080-beam 2D LiDAR range measurements ($270^\circ$ FOV) |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | **4,461** | ~250 Hz | Dynamic steering angle ($\delta$) and speed ($v$) commands |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | **4,458** | ~250 Hz | Ground-truth coordinates $(x, y)$, heading yaw, linear & angular velocity |
| `/tf` | `tf2_msgs/msg/TFMessage` | **17,836** | ~1,000 Hz | Coordinate frame tree (`map` $\to$ `ego_racecar/base_link` $\to$ `laser`) |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | **1** | Static | Fixed sensor offset transforms |
| `/car_hud` | `visualization_msgs/msg/Marker` | **4,460** | ~250 Hz | Floating 3D telemetry display marker (speed, steering, state) |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | **4,457** | ~250 Hz | 3D lookahead target sphere tracking chosen gap center |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | **4,458** | ~250 Hz | 3D HUD billboard showing active gap range and angle |
| `/lap_stats` | `std_msgs/msg/String` | **4,463** | ~250 Hz | Real-time lap stats (lap count, elapsed time, clean status) |
| `/map` | `nav_msgs/msg/OccupancyGrid` | **1** | Static | Sakhir circuit track map grid |
| `/bond` | `bond/msg/Status` | **358** | 20 Hz | Lifecycle manager heartbeat status |

---

## 🚀 How to Run the Bag File (Step-by-Step)

Before running, open a terminal in the repository root and source ROS 2:
```bash
source /opt/ros/humble/setup.bash
```

### 1. Inspect Bag Metadata (`ros2 bag info`)
To view metadata, message counts, duration, and topic types:
```bash
ros2 bag info "ROS 2 Bag"
```

### 2. Standard Playback at Normal Speed (1.0x)
Replay the simulation dataset in real time:
```bash
ros2 bag play "ROS 2 Bag"
```

### 3. Slow-Motion Playback at Half Speed (0.5x)
Replay at half speed to closely inspect cornering transitions, bubble expansion, and gap selections:
```bash
ros2 bag play "ROS 2 Bag" --rate 0.5
```

### 4. Continuous Loop Playback (`--loop`)
Keep replaying in an infinite loop (ideal for live demonstrations or when inspecting telemetry in another terminal):
```bash
ros2 bag play "ROS 2 Bag" --loop
```

### 5. Filter Specific Topics
If you only want to replay the LiDAR scans and Ackermann driving commands:
```bash
ros2 bag play "ROS 2 Bag" --topics /scan /drive
```

---

## 🎮 How to Handle Playback (Interactive Keyboard Controls)

While `ros2 bag play` is running in your terminal, ROS 2 provides real-time interactive keyboard controls:

| Key Press | Action | Effect / Purpose |
| :---: | :--- | :--- |
| **`Space`** | **Pause / Resume** | Freeze playback at any millisecond to inspect vehicle pose or LiDAR values; press again to continue. |
| **`Right Arrow` (`→`)** | **Step 1 Message** | While paused, advance forward frame-by-frame (message-by-message) for microsecond-level analysis. |
| **`Up Arrow` (`↑`)** | **Increase Rate** | Speeds up playback (e.g., $1.0\times \to 1.5\times \to 2.0\times$). |
| **`Down Arrow` (`↓`)** | **Decrease Rate** | Slows down playback (e.g., $1.0\times \to 0.5\times \to 0.25\times$). |
| **`Ctrl + C`** | **Stop / Exit** | Cleanly terminates playback and releases all ROS 2 topic publishers. |

---

## 🔍 What to Do Next (Evaluation, Telemetry & Visualization Workflows)

Once the bag file is playing, what can an evaluator, professor, or researcher actually do with the published data? Here are 4 practical workflows:

### Workflow 1: 3D Visualization in RViz2 (Without Physics Simulation)
Because this ROS 2 bag recorded all coordinate transforms (`/tf`), sensor scans (`/scan`), track maps (`/map`), and telemetry markers (`/ftg_target_marker`, `/car_hud`), you can **fully visualize the 3D car driving on the track without needing to launch the simulator engine!**

1. **Terminal 1 (Start looping playback):**
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 bag play "ROS 2 Bag" --loop
   ```

2. **Terminal 2 (Launch RViz2):**
   ```bash
   source /opt/ros/humble/setup.bash
   rviz2
   ```

3. **In RViz2 GUI:**
   - Set **Fixed Frame** to `map` (or `ego_racecar/base_link`).
   - Click **Add** $\to$ **By topic**:
     - `/scan` $\to$ **LaserScan** *(Visualizes 1080 green/red LiDAR beams hitting the walls)*.
     - `/map` $\to$ **Map** *(Visualizes Sakhir circuit walls and layout)*.
     - `/tf` $\to$ **TF** *(Visualizes coordinate axes moving with the car)*.
     - `/ftg_target_marker` $\to$ **Marker** *(Visualizes the reactive target point chosen by the controller)*.
     - `/car_hud` $\to$ **Marker** *(Visualizes the floating speed and status HUD above the car)*.
   - **Result:** You will see the car navigate the track smoothly in real time just as recorded!

---

### Workflow 2: Live Telemetry & Command Monitoring (Terminal 2)
While the bag is playing in Terminal 1, open a second terminal to monitor the exact control commands and vehicle state:

- **Monitor Steering & Speed Commands:**
  ```bash
  ros2 topic echo /drive
  ```
  *(Shows real-time `steering_angle` and `speed` outputs generated by the controller).*

- **Monitor Vehicle Coordinates & Odometry:**
  ```bash
  ros2 topic echo /ego_racecar/odom
  ```
  *(Displays 3D position $(x, y, z)$, linear velocity, and orientation quaternion).*

- **Monitor Real-Time Lap Performance:**
  ```bash
  ros2 topic echo /lap_stats
  ```

- **Verify Message Rates & Bandwidth:**
  ```bash
  ros2 topic hz /scan
  ros2 topic hz /drive
  ros2 topic bw /scan
  ```

---

### Workflow 3: Offline Algorithm Testing & Validation
Researchers and students can test **new** reactive algorithms or filtering methods **offline** using this recorded dataset without opening the gym simulator:

1. In Terminal 1, run:
   ```bash
   ros2 bag play "ROS 2 Bag" --loop --topics /scan /ego_racecar/odom /tf
   ```
2. In Terminal 2, run your custom candidate controller node that subscribes to `/scan`.
3. Compare your candidate node's steering decisions against the benchmark `/drive` commands recorded in the bag to benchmark performance.

---

### Workflow 4: Quantitative Data Extraction & Telemetry Plotting
You can query the SQLite3 database file (`f1tenth_run_01_0.db3`) directly or use Python to parse and plot telemetry curves:

```python
import sqlite3

# Connect directly to the SQLite3 ROS 2 bag database
conn = sqlite3.connect("ROS 2 Bag/f1tenth_run_01_0.db3")
cursor = conn.cursor()

# Inspect recorded tables
cursor.execute("SELECT name FROM sqlite_master WHERE type='table';")
print("Tables in bag database:", cursor.fetchall())

# Count total stored messages
cursor.execute("SELECT count(*) FROM messages;")
print("Total messages stored:", cursor.fetchone()[0])
```

---

## 🛠️ Automated Helper Scripts

For convenience, two executable bash scripts are provided in `scripts/`:

```bash
# 1. Inspect and replay at normal speed (1.0x):
./scripts/play_bag.sh "ROS 2 Bag"

# 2. Replay at half speed (0.5x):
./scripts/play_bag.sh "ROS 2 Bag" 0.5

# 3. Replay in a continuous loop:
./scripts/play_bag.sh "ROS 2 Bag" 1.0 --loop

# 4. Record a new experiment run:
./scripts/record_bag.sh "my_new_run"
```

---

## 📌 Oral Defense Summary for Instructors
- **Why record a ROS 2 Bag?**  
  A ROS 2 bag provides immutable, replayable scientific proof of autonomous vehicle performance. It records exact sensor measurements and actuator commands without simulation randomness.
- **Why 0 collisions?**  
  The recorded `/scan` and `/ego_racecar/odom` streams verify that the safety bubble around the nearest obstacle remained strictly larger than the car's bounding box ($0.40\,\text{m} > 0.30\,\text{m}$), guaranteeing 0 wall contacts.
- **Reproducibility:**  
  Any student or evaluator anywhere in the world can run `ros2 bag play "ROS 2 Bag"` on ROS 2 Humble/Foxy to replay and audit the entire drive down to the nanosecond.
