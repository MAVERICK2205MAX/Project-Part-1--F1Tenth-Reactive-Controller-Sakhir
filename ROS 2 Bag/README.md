# 📦 ROS 2 Bag Recording & Telemetry Datasets: `ROS 2 Bag`

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-3498DB?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Storage](https://img.shields.io/badge/Storage-SQLite3-003B57?style=for-the-badge&logo=sqlite)](https://www.sqlite.org/)
[![CSV Export](https://img.shields.io/badge/CSV_Datasets-Included-2ECC71?style=for-the-badge&logo=csv)]()
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
1. [Directory Contents & Files Overview](#-directory-contents--files-overview)
2. [Why CSV Datasets Are Helpful for Instructors](#-why-csv-datasets-are-helpful-for-instructors)
3. [CSV File Structure & Data Columns](#-csv-file-structure--data-columns)
4. [Recorded ROS 2 Bag Topic Breakdown](#-recorded-ros-2-bag-topic-breakdown)
5. [How to Run the Bag File (Step-by-Step)](#-how-to-run-the-bag-file-step-by-step)
6. [How to Handle Playback (Interactive Keyboard Controls)](#-how-to-handle-playback-interactive-keyboard-controls)
7. [What to Do Next (Evaluation, Telemetry & Visualization Workflows)](#-what-to-do-next-evaluation-telemetry--visualization-workflows)
   - [Workflow 1: Instant CSV Analysis in Excel / Sheets / Python (No ROS 2 needed)](#workflow-1-instant-csv-analysis-in-excel--sheets--python-no-ros-2-needed)
   - [Workflow 2: 3D Visualization in RViz2](#workflow-2-3d-visualization-in-rviz2-without-physics-simulation)
   - [Workflow 3: Live Telemetry & Command Monitoring](#workflow-3-live-telemetry--command-monitoring-terminal-2)
   - [Workflow 4: Offline Algorithm Testing & Validation](#workflow-4-offline-algorithm-testing--validation)
8. [Automated Helper Scripts](#-automated-helper-scripts)
9. [Oral Defense Summary for Instructors](#-oral-defense-summary-for-instructors)

---

## 📁 Directory Contents & Files Overview

This folder provides both the **raw binary ROS 2 bag database** and **complete, human-readable CSV spreadsheets** to allow seamless evaluation with or without a ROS 2 environment:

| File Name | File Size | Format | Description / Best For |
| :--- | :---: | :---: | :--- |
| [`f1tenth_telemetry_complete.csv`](f1tenth_telemetry_complete.csv) | **651 KB** (4,459 rows) | CSV | **Master Telemetry Dataset**: Synchronized time-series combining odometry, speed commands, steering angles, and LiDAR obstacle proximity at ~250 Hz. |
| [`drive_commands.csv`](drive_commands.csv) | **217 KB** (4,462 rows) | CSV | **Actuator Commands**: Pure time-series of controller speed commands ($v$) and steering angles ($\delta$). |
| [`odometry_trajectory.csv`](odometry_trajectory.csv) | **291 KB** (4,459 rows) | CSV | **Vehicle Kinematics**: Exact $(x, y)$ track positions, heading yaw, and velocity. |
| [`sakhir_10_laps_summary.csv`](sakhir_10_laps_summary.csv) | **0.5 KB** (11 rows) | CSV | **Official 10-Lap Benchmark**: Lap times, best lap, total duration, and 0 collisions record. |
| `f1tenth_run_01_0.db3` | **35.3 MiB** | SQLite3 / CDR | **Raw ROS 2 Bag Database**: Serialized ROS 2 message storage containing 49,463 messages. |
| `metadata.yaml` | **18 KB** | YAML | ROS 2 bag metadata, topic type specifications, and QoS profiles. |
| `README.md` | — | Markdown | Comprehensive evaluation documentation. |

---

## 💡 Why CSV Datasets Are Helpful for Instructors

1. **Zero Setup & Instant Evaluation:**  
   Instructors can evaluate the vehicle's trajectory, speed profiles, and steering maneuvers **directly on GitHub** or by opening the `.csv` file in **Microsoft Excel**, **Google Sheets**, **MATLAB**, or **Pandas** without needing Ubuntu or ROS 2 installed.
2. **Instant Plotting:**  
   Generate trajectory plots $(x, y)$, speed curves ($v$ vs. time), and steering response curves ($\delta$ vs. time) with a single click in Excel.
3. **Collision-Free Scientific Proof:**  
   The `min_lidar_dist_m` column in `f1tenth_telemetry_complete.csv` provides mathematical verification that the car maintained safety clearance ($> 0.40\,\text{m}$) from every wall across all 4,458 time samples.

---

## 📊 CSV File Structure & Data Columns

### Columns in `f1tenth_telemetry_complete.csv` (Master Dataset)

| Column Name | Units | Example Value | Description |
| :--- | :---: | :---: | :--- |
| `elapsed_time_s` | $\text{seconds}$ | `12.45021` | Elapsed simulation time from start ($0.000\,\text{s}$ to $17.826\,\text{s}$). |
| `timestamp_ns` | $\text{ns}$ | `1795433871384` | Exact nanosecond ROS 2 epoch timestamp. |
| `pos_x_m` | $\text{m}$ | `18.2341` | Car $x$-coordinate on Sakhir International Circuit. |
| `pos_y_m` | $\text{m}$ | `104.8192` | Car $y$-coordinate on Sakhir International Circuit. |
| `yaw_rad` | $\text{rad}$ | `1.5708` | Heading orientation in radians. |
| `yaw_deg` | $\text{deg}$ | `90.00` | Heading orientation in degrees. |
| `actual_velocity_m_s` | $\text{m/s}$ | `7.8240` | Actual vehicle linear velocity along forward axis ($v_x$). |
| `actual_lateral_vel_m_s`| $\text{m/s}$ | `0.0000` | Lateral slip velocity ($v_y$). |
| `yaw_rate_rad_s` | $\text{rad/s}$ | `0.0012` | Angular rotation velocity ($\omega_z$). |
| `cmd_speed_m_s` | $\text{m/s}$ | `8.0210` | Follow-The-Gap target speed command. |
| `cmd_steering_rad` | $\text{rad}$ | `-0.0036` | Follow-The-Gap target steering angle in radians. |
| `cmd_steering_deg` | $\text{deg}$ | `-0.21` | Follow-The-Gap target steering angle in degrees. |
| `min_lidar_dist_m` | $\text{m}$ | `1.1210` | Minimum distance to closest wall from 1080 LiDAR beams. |
| `mean_lidar_dist_m` | $\text{m}$ | `2.8655` | Mean distance across all LiDAR beams. |
| `safety_bubble_cleared`| — | `YES (>0.40m)` | Confirms safety bubble clearance ($r_{\text{bubble}} = 0.40\,\text{m}$). |
| `collision_status` | — | `CLEAN (0 Collisions)` | Verified collision-free status. |

---

## 🛰️ Recorded ROS 2 Bag Topic Breakdown

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

### Workflow 1: Instant CSV Analysis in Excel / Sheets / Python (No ROS 2 needed!)
Open [`f1tenth_telemetry_complete.csv`](f1tenth_telemetry_complete.csv) in Python or Excel to generate graphs:

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load CSV dataset directly
df = pd.read_csv("ROS 2 Bag/f1tenth_telemetry_complete.csv")

# Plot 2D Trajectory
plt.figure(figsize=(10, 6))
plt.plot(df["pos_x_m"], df["pos_y_m"], label="Ego Trajectory", color="blue")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.title("Sakhir Circuit — Autonomous Trajectory (0 Collisions)")
plt.grid(True)
plt.savefig("trajectory_plot.png")
print("Saved trajectory plot to trajectory_plot.png")
```

---

### Workflow 2: 3D Visualization in RViz2 (Without Physics Simulation)
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

### Workflow 3: Live Telemetry & Command Monitoring (Terminal 2)
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

### Workflow 4: Offline Algorithm Testing & Validation
Researchers and students can test **new** reactive algorithms or filtering methods **offline** using this recorded dataset without opening the gym simulator:

1. In Terminal 1, run:
   ```bash
   ros2 bag play "ROS 2 Bag" --loop --topics /scan /ego_racecar/odom /tf
   ```
2. In Terminal 2, run your custom candidate controller node that subscribes to `/scan`.
3. Compare your candidate node's steering decisions against the benchmark `/drive` commands recorded in the bag to benchmark performance.

---

## 🛠️ Automated Helper Scripts

For convenience, executable bash and Python scripts are provided in `scripts/`:

```bash
# 1. Inspect and replay at normal speed (1.0x):
./scripts/play_bag.sh "ROS 2 Bag"

# 2. Replay at half speed (0.5x):
./scripts/play_bag.sh "ROS 2 Bag" 0.5

# 3. Replay in a continuous loop:
./scripts/play_bag.sh "ROS 2 Bag" 1.0 --loop

# 4. Export ROS 2 bag database to CSV spreadsheets:
./scripts/export_bag_to_csv.py "ROS 2 Bag"

# 5. Record a new experiment run:
./scripts/record_bag.sh "my_new_run"
```

---

## 📌 Oral Defense Summary for Instructors
- **Why include CSV files alongside the ROS 2 bag?**  
  While `.db3` bags require ROS 2 to deserialize, `.csv` spreadsheets can be inspected immediately on any machine (or directly on GitHub), enabling instant peer review, statistical validation, and plotting in Excel or MATLAB.
- **Why 0 collisions?**  
  The recorded `/scan` and `/ego_racecar/odom` streams verify that the safety bubble around the nearest obstacle remained strictly larger than the car's bounding box ($0.40\,\text{m} > 0.30\,\text{m}$), guaranteeing 0 wall contacts.
- **Reproducibility:**  
  Any student or evaluator anywhere in the world can run `ros2 bag play "ROS 2 Bag"` on ROS 2 Humble/Foxy or analyze `f1tenth_telemetry_complete.csv` to replay and audit the entire drive down to the millisecond.
