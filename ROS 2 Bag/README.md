# 📦 ROS 2 Bag Recording: `f1tenth_run_01`

**Course:** Autonomous Vehicles — II PAO 2026  
**Project:** Part 1 — Reactive Controller Competition (Sakhir Track)  
**Student:** ISLAM KAZI MAJHARUL (Group 2)  
**Storage Format:** SQLite3 (`sqlite3`)  
**Bag Status:** Verified & Complete  

---

## 📊 Recorded Bag Overview

This folder contains the complete serialized simulation dataset recorded with:
```bash
ros2 bag record -a -o f1tenth_run_01
```

It captures all LiDAR scans, vehicle commands, transforms, and telemetry messages with nanosecond timestamps.

| Property | Value |
| :--- | :--- |
| **Directory Name** | `ROS 2 Bag/` (alias: `f1tenth_run_01/`) |
| **Primary Database File** | `f1tenth_run_01_0.db3` (35.3 MiB) |
| **Metadata File** | `metadata.yaml` |
| **Duration** | **17.83 s** |
| **Total Message Count** | **49,463 messages** |
| **Track** | Sakhir International Circuit |
| **Controller** | Follow-The-Gap with Speed-Scheduled Gain |

---

## 🛰️ Recorded Topic Breakdown

| Topic | Message Type | Message Count | Description |
| :--- | :--- | :---: | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | **4,463** | 1080-beam 2D LiDAR range measurements |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | **4,461** | Steering angle and speed commands |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | **4,458** | Vehicle position $(x, y)$ and velocity |
| `/tf` | `tf2_msgs/msg/TFMessage` | **17,836** | Coordinate frame transforms |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | **1** | Static sensor transforms |
| `/car_hud` | `visualization_msgs/msg/Marker` | **4,460** | Floating 3D telemetry display marker |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | **4,457** | 3D lookahead target sphere |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | **4,458** | HUD status billboard |
| `/lap_stats` | `std_msgs/msg/String` | **4,463** | Real-time lap stats string |
| `/map` | `nav_msgs/msg/OccupancyGrid` | **1** | Sakhir circuit track map grid |
| `/bond` | `bond/msg/Status` | **358** | Lifecycle manager bond status |

---

## 💻 Instructions for Instructor to Inspect & Play

### 1. Inspect Bag Details (`ros2 bag info`)
```bash
ros2 bag info f1tenth_run_01
```

### 2. Replay the Bag at Full Speed (`ros2 bag play`)
```bash
ros2 bag play f1tenth_run_01
```

### 3. Replay at Half Speed (0.5x)
```bash
ros2 bag play f1tenth_run_01 --rate 0.5
```

### 4. Replay Using Helper Script
```bash
./scripts/play_bag.sh f1tenth_run_01 0.5
```

*(Interactive playback controls: `Space` = Pause/Resume, `Right Arrow` = Step 1 message, `Up/Down` = Change rate)*.
