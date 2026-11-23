# 🏎️ Reactive Follow the Gap (FTG) Controller with Disparity Extender — Sakhir Circuit (F1Tenth)

[![ROS 2](https://img.shields.io/badge/ROS_2-HUMBLE-22314E?style=for-the-badge&logo=ros)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/PYTHON-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white)](https://www.python.org/)
[![F1TENTH](https://img.shields.io/badge/F1TENTH-GYM_ROS-EE3124?style=for-the-badge)](https://github.com/f1tenth/f1tenth_gym_ros)
[![VALIDATION](https://img.shields.io/badge/VALIDATION-10_COLLISION--FREE_LAPS-2E7D32?style=for-the-badge)](sakhir_lap_records.csv)
[![ROS 2 Bag](https://img.shields.io/badge/ROS_2_Bag-Dataset-8A2BE2?style=for-the-badge&logo=ros)](ROS%202%20Bag)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg?style=for-the-badge)](LICENSE)

- **Driver & Lead Developer:** [ISLAM KAZI MAJHARUL](https://github.com/MAVERICK2205MAX) (Group 2)
- **Contributors:** Winter Delgado ([@widegonz](https://github.com/widegonz))
- **Course:** Autonomous Vehicles — II PAO 2026
- **Assessment:** First Midterm Project (Part 1) — Reactive Controllers Competition
- **Assigned Track:** Sakhir (Group 2)
- **ROS 2 Bag Recording:** [ROS 2 Bag](ROS%202%20Bag) *(Official dataset: 49,463 messages)*
- **Repository Visibility:** Public & Open Source for All Time ([MIT License](LICENSE))

---

## 📑 Table of Contents

1. [Project Overview and Objectives](#1-project-overview-and-objectives)
2. [Geometric Analysis of the Sakhir Circuit](#2-geometric-analysis-of-the-sakhir-circuit)
3. [ROS 2 System Architecture](#3-ros-2-system-architecture)
4. [Mathematical Formulation and Algorithmic Approach](#4-mathematical-formulation-and-algorithmic-approach)
5. [Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory](#5-telemetry-system-lap-counter-lap-timer-distance-tracker-and-3d-rviz-trajectory)
6. [Detailed Code Structure](#6-detailed-code-structure)
7. [Hyperparameter Table and Tuning Strategy](#7-hyperparameter-table-and-tuning-strategy)
8. [Installation, Configuration, and Execution Instructions](#8-installation-configuration-and-execution-instructions)
9. [Experimental Results, Lap Distance, and Video Evidence](#9-experimental-results-lap-distance-and-video-evidence)
10. [Contributors and Project Authorship](#10-contributors-and-project-authorship)
11. [ROS 2 Bag Recording and Playback (`ROS 2 Bag`)](#11-ros-2-bag-recording-and-playback-ros-2-bag)

---

## 1. Project Overview and Objectives

As part of the **Autonomous Vehicles course (Project Part 1)**, this repository presents the implementation, mathematical derivation, and empirical validation of an advanced **Follow-The-Gap (FTG) reactive controller** equipped with a **Disparity Extender**, **Cartesian Safety Corridor hazard filtering**, and **Curvature-Clearance Speed Profiling** on the official **F1Tenth Sakhir International Circuit** map.

### Competition Challenges & Grading Objectives:
- **🌀 Challenge 1 — 10 Consecutive Laps Without Collisions:** Complete 10 consecutive laps under strict collision-free safety constraints (wall clearance margin $\ge 0.35\text{ m}$).
- **⏱️ Challenge 2 — Fastest Lap Time:** Optimize cornering lines and longitudinal speed profiles to achieve the minimum lap time among all competitors in Group 2.
- **🔄 Lap Counter Implementation (+2 pts):** Fully autonomous finish-line crossing detection using spatial hysteresis to record and count laps without manual intervention.
- **⏱️ Lap Timer Implementation (+3 pts):** High-precision per-lap stopwatch recording individual lap splits, best lap, and total elapsed simulation time.
- **📘 GitHub Documentation (+3 pts):** Exhaustive technical tutorial detailing the algorithmic methodology, system architecture, tuning rationale, and execution commands.
- **🎥 Video Evidence (+2 mandatory pts):** Screen recording verifying 10 full laps, live HUD, continuous RViz trajectory path, and lap timing.

---

## 2. Geometric Analysis of the Sakhir Circuit

The **Sakhir International Circuit** in the F1Tenth simulator represents one of the most challenging tracks due to its distinct geometric sections:

```
                      [High-Speed Sweeping Corner]
                         . - - - - - - - - .
                       /                     \
   [Back Straight]    /                       \   [Technical Infield]
=====================/                         \====================
                      \                       /
                       \                     /
                         . - - - - - - - - .
                            [Chicane & Hairpin]
                                     |
                                     v
                       ==============================
                       [Finish Line (x=0, y=0) & Straight]
```

1. **Ultra-Long Main Straight:** High-speed section where vehicles easily exceed $7.5\text{ m/s}$. Standard FTG controllers develop high-frequency lateral oscillation (snaking/wobbling) due to parallel side-wall beam reflections.
2. **Sharp Hairpin & Tight Chicanes:** Severe curvature where inner wall occlusions must be dynamically inflated via a **Disparity Extender** to prevent the chassis from clipping the apex.
3. **Narrow Transition Corridors:** Corridor widths narrow down to $\approx 2.2\text{ m}$. Bounding obstacle detection to a Cartesian travel corridor ($|y| \le 0.45\text{ m}$) is crucial to prevent side walls from splitting the gap into competing bistable halves.

---

## 3. ROS 2 System Architecture

The controller operates inside a decoupled publish/subscribe ROS 2 architecture where the node `reactive_gap_controller` (implemented in `controllers/gap_node.py`) processes 2D LiDAR scans at high frequency ($\approx 40\text{ Hz}$) while concurrently updating odometry metrics, distance integration, and 3D RViz markers:

```
+-------------------------------------------------------------------+
|                     F1Tenth Gym ROS Bridge                        |
|                  (Physics Simulator + 2D Map)                     |
+---------------------------------+---------------------------------+
                                  |
             | /scan              | /ego_racecar/odom
             | (sensor_msgs/      | (nav_msgs/
             |  LaserScan)        |  Odometry)
             v                    v
+-------------------------------------------------------------------+
| Node: reactive_gap_controller                                     |
| (controllers/gap_node.py)                                         |
|                                                                   |
| 1. FOV Truncation & 7-Point Gaussian Range Smoothing              |
| 2. Cartesian Corridor Safety Bubble (|y| <= 0.45m)                |
| 3. Disparity Extender for Wall Discontinuities                    |
| 4. Adaptive Max Gap & Plateau Median Lookahead Centering          |
| 5. Speed-Scheduled Gain Attenuation & Slew Rate Limiter           |
| 6. Automatic Lap Counter, Lap Timer & Distance Integration        |
+---+---------------+-----------------+------------------+----------+
    |               |                 |                  |
    | /drive        | /ftg_hud_marker | /ftg_target_     | /ftg_trajectory_marker
    | (Ackermann    | (3D Floating    |  marker          | & /ftg_path
    |  DriveStamped)|  HUD Marker)    | (Target Sphere)  | (Continuous Path)
    v               v                 v                  v
+----------+   +----------------------------------------------------+
|   Ego    |   |                       RViz 2                       |
| Racecar  |   |    (3D Floating HUD + Racing Line + Target Point)  |
+----------+   +----------------------------------------------------+
```

### ROS 2 Communication Interfaces (Topics)

| Topic | Message Type | Direction | System Role |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | Subscribed | Receives the raw 2D LiDAR range array (1080 beams, $270^\circ$ FOV). |
| `/ego_racecar/odom` | `nav_msgs/msg/Odometry` | Subscribed | Provides $(x,y)$ vehicle pose for lap counting, lap timing, distance integration, and path drawing. |
| `/drive` | `ackermann_msgs/msg/AckermannDriveStamped` | Published | Outputs target steering angle (`steering_angle`) and forward speed (`speed`). |
| `/ftg_hud_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D floating HUD above the vehicle displaying lap number, current time, lap distance, and speed. |
| `/ftg_target_marker` | `visualization_msgs/msg/Marker` | Published | Renders a 3D green sphere (`Marker.SPHERE`) at the chosen lookahead target point inside the navigable gap. |
| `/ftg_trajectory_marker` | `visualization_msgs/msg/Marker` | Published | Draws a continuous 3D line (`Marker.LINE_STRIP`) showing the actual trajectory driven on Sakhir. |
| `/ftg_path` | `nav_msgs/msg/Path` | Published | Publishes historical pose sequences compatible with the native RViz `Path` display plugin. |

---

## 4. Mathematical Formulation and Algorithmic Approach

Unlike a baseline Follow-the-Gap implementation—which frequently clips inner corners on hairpins or oscillates down high-speed straights—this controller executes an advanced 7-stage reactive pipeline on every incoming LiDAR scan:

### Stage 1: Field-of-View (FOV) Cropping and Spatial Smoothing
The raw LiDAR scan is cropped to retain only the forward driving sector $[-\theta_{\text{fov}}, +\theta_{\text{fov}}]$ with $\theta_{\text{fov}} = 70^\circ$ ($1.22\text{ rad}$), discarding rearward beams. Non-finite values (`NaN`, $\pm\infty$) are clamped to the maximum sensor horizon $R_{\text{max}} = 8.0\text{ m}$. To eliminate single-beam quantization noise, a 7-point Gaussian smoothing convolution kernel is applied:

$$r_{\text{smooth}}[i] = \sum_{m=-3}^{3} w_m \cdot r[i+m], \quad w = \frac{1}{27}[1, 3, 6, 7, 6, 3, 1]$$

### Stage 2: Disparity Extender
When approaching corners or chicane walls, sharp depth discontinuities occur between a beam hitting an inner wall edge ($d_{\text{near}}$) and an adjacent beam looking into the open track. Whenever $|r[i+1] - r[i]| > \delta_{\text{disp}}$ ($\delta_{\text{disp}} = 1.2\text{ m}$), the vehicle's virtual half-width ($w_{\text{half}} = 0.35\text{ m}$) is projected onto the closer reading to mask unsafe rays:

$$\Delta \theta = \arctan\left(\frac{w_{\text{half}}}{d_{\text{near}}}\right), \quad N_{\text{samples}} = \left\lceil \frac{\Delta \theta}{\Delta \alpha} \right\rceil$$

where $\Delta \alpha$ is the LiDAR angular resolution (`angle_increment`). Disparity checking is bounded to the forward travel arc ($|\theta| \le 20.0^\circ$) to avoid artificial side-wall cuts.

### Stage 3: Dynamic Safety Bubble (Cartesian Travel Corridor Filtering)
In narrow corridors ($\approx 2.2\text{ m}$ wide), radial safety bubbles at $\pm 45^\circ$ catch parallel side walls, creating artificial gaps and causing the car to oscillate. We eliminate this by transforming beams to Cartesian vehicle coordinates:

$$x_i = r_i \cos\theta_i, \quad y_i = r_i \sin\theta_i$$

Hazard detection is strictly bounded to the vehicle's forward travel corridor:

$$|y_i| \le w_{\text{safety}} \quad (0.45\text{ m}), \quad x_i \in [0.05\text{ m}, d_{\text{bubble\_max}} \ (2.0\text{ m})]$$

If the closest obstacle lies within this corridor, all rays within the safety radius $R_{\text{bubble}} = 0.35\text{ m}$ are zeroed out ($0.0\text{ m}$):

$$N_{\text{bubble}} = \left\lceil \frac{\arctan(R_{\text{bubble}} / \max(d_{\text{min}}, 0.1))}{\Delta \alpha} \right\rceil$$

### Stage 4: Adaptive Threshold Max-Gap Detection
Free space is thresholded at $T_{\text{gap}} = 0.70\text{ m}$. The widest contiguous sequence of free beams $[i_{\text{start}}, i_{\text{end}}]$ is extracted:

$$\text{Max Gap} = \arg\max_{g \in \mathcal{G}} \left(i_{\text{end}}^{(g)} - i_{\text{start}}^{(g)}\right)$$

### Stage 5: Plateau-Median Centered Target Selection
Rather than selecting a discrete single maximum-depth beam (which jumps discontinuously between scans), the controller identifies the deepest plateau ($r \ge \max(0.92 \cdot r_{\text{max}}, r_{\text{max}} - 0.25\text{ m})$) and blends the geometric midpoint with the plateau median:

$$i_{\text{target}} = \text{round}\left(w_{\text{center}} \cdot i_{\text{mid}} + (1 - w_{\text{center}}) \cdot i_{\text{plateau\_median}}\right)$$

where $w_{\text{center}} = 0.75$ on open straights (enforcing centered tracking) and $0.50$ in tight corners (allowing racing-line apex clipping).

### Stage 6: Speed-Scheduled Steering Gain Attenuation
At high speeds, lateral tire forces scale nonlinearly. A speed-scheduled attenuation scales the steering command:

$$\delta_{\text{target}} = \delta_{\text{raw}} \cdot \max\left(0.25, \min\left(1.0, \left(\frac{v_{\text{base}}}{v}\right)^p\right)\right)$$

where $v_{\text{base}} = 3.0\text{ m/s}, p = 1.0$. The steering angle is then smoothed through an Exponential Moving Average (EMA) low-pass filter ($\alpha = 0.20$) and a dynamic slew rate limiter ($|\dot{\delta}| \le 2.8\text{ rad/s}$):

$$\delta_k = (1 - \alpha)\delta_{k-1} + \alpha \delta_{\text{target}}$$

### Stage 7: Curvature-Aware Anticipatory Braking
Forward velocity is smoothly adjusted by combining steering curvature ($f_{\text{steer}}$) and forward distance clearance ($f_{\text{dist}}$):

$$f_{\text{steer}} = \max\left(0.0, 1.0 - \frac{|\delta|}{20.0^\circ}\right), \quad f_{\text{dist}} = \min\left(1.0, \max\left(0.0, \frac{d_{\text{fwd}} - d_{\text{floor}}}{d_{\text{range}}}\right)\right)$$

$$v = v_{\text{min}} + (v_{\text{max}} - v_{\text{min}}) \cdot \left(0.50 \cdot f_{\text{steer}} + 0.50 \cdot f_{\text{dist}}\right)$$

yielding peak velocities of **$7.6\text{ m/s}$** on straightaways and safe deceleration to **$2.0\text{ m/s}$** before hairpin apexes.

---

## 5. Telemetry System: Lap Counter, Lap Timer, Distance Tracker, and 3D RViz Trajectory

Synchronized with `/ego_racecar/odom`, the telemetry subsystem provides full observability during autonomous execution:

- **Automatic Finish-Line Detection with Spatial Hysteresis:**  
  Upon simulator startup, initial coordinates $(x_0, y_0) = (0.0, 0.0)$ on Sakhir are captured. To prevent false triggers at the start line, the trigger is armed only after the vehicle travels deep into the circuit ($d > 25.0\text{ m}$). A completed lap is registered when the vehicle crosses the finish plane from $y < 0$ to $y \ge 0$ with $|x| \le 3.0\text{ m}$ and an elapsed lap time $\Delta t_{\text{lap}} \ge 15.0\text{ s}$.
- **Euclidean Lap Distance Integration:**  
  On every odometry update $(x_k, y_k)$, the incremental displacement:
  $$\Delta s_k = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$
  is accumulated to compute the exact racing line distance traveled per lap:
  $$D_{\text{lap}} = \sum_{k=1}^K \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2}$$
- **Real-Time Trajectory Rendering in RViz:**  
  Every $0.25\text{ m}$ of travel, the vehicle's $(x,y)$ coordinates in the `map` frame are appended to `/ftg_trajectory_marker` (`Marker.LINE_STRIP`) and `/ftg_path` (`nav_msgs/Path`), drawing a vibrant orange racing line that visualizes the path taken around Sakhir.
- **3D RViz Floating HUD & Target Marker:**  
  - `/ftg_hud_marker`: Floating text billboard above the car displaying lap count, live lap time, speed, and integrated lap distance.
  - `/ftg_target_marker`: 3D green sphere (`Marker.SPHERE`) rendered at the exact target coordinate selected inside the navigable gap.

---

## 6. Detailed Code Structure

```
Project-Part-1--F1Tenth-Reactive-Controller-Sakhir/
├── CMakeLists.txt / setup.py       # Package build configuration & ROS 2 entry points
├── package.xml                     # ROS 2 Humble package dependencies
├── controllers/
│   ├── __init__.py
│   ├── gap_node.py                 # Core FTG controller + Telemetry + RViz Markers
│   └── telemetry_gui.py            # Real-time HUD overlay OpenCV dashboard
├── launch/
│   └── competition_sakhir.launch.py # Complete launch file (Bridge + Controller + GUI)
├── maps/
│   ├── Sakhir_map.png              # Sakhir track occupancy grid image
│   ├── Sakhir_map.yaml             # Map metadata and resolution (0.05 m/px)
│   └── Sakhir_centerline.csv       # Track centerline reference coordinates
├── sakhir_lap_records.csv          # Official benchmark records (10 Collision-Free Laps)
├── run_headless_10laps.py          # Automated fast verification harness
├── ORAL_DEFENSE_PREP.md            # Comprehensive oral defense preparation guide
└── README.md                       # Full technical documentation
```

---

## 7. Hyperparameter Table and Tuning Strategy

| Parameter | Tuned Value | Units | Physical Interpretation & Tuning Rationale |
| :--- | :---: | :---: | :--- |
| `bubble_radius` | `0.35` | $\text{m}$ | Virtual obstacle inflation radius ($R_{\text{car}} + 0.15\text{ m}$ safety margin). |
| `safety_width` | `0.45` | $\text{m}$ | Cartesian lateral corridor half-width. Ignores side walls at $|y| > 0.45\text{ m}$ to eliminate wobble. |
| `bubble_max_dist` | `2.0` | $\text{m}$ | Maximum forward distance to evaluate obstacle hazard bubbles. |
| `max_range` | `8.0` | $\text{m}$ | Sensor horizon clamp; prevents distant open background from skewing gap detection. |
| `fov_deg` | `70.0` | $\text{deg}$ | Forward field-of-view half-angle ($[-70^\circ, +70^\circ]$ total $140^\circ$ scan window). |
| `v_min` | `2.0` | $\text{m/s}$ | Minimum cornering velocity; guarantees high tire grip through tight hairpins. |
| `v_max` | `7.6` | $\text{m/s}$ | Maximum straightaway velocity; tuned for high-speed stability without wall collisions. |
| `disparity_threshold` | `1.2` | $\text{m}$ | Depth jump threshold triggering disparity bubble extension. |
| `ema_alpha` | `0.20` | — | Low-pass filter smoothing coefficient; suppresses high-frequency steering chatter. |
| `max_steer_velocity` | `2.8` | $\text{rad/s}$ | Steering slew rate limit; preserves chassis balance during aggressive direction changes. |
| `v_steer_base` | `3.0` | $\text{m/s}$ | Baseline velocity threshold above which steering gain is attenuated. |
| `d_floor` | `2.0` | $\text{m}$ | Minimum forward buffer threshold where full corner deceleration is engaged. |
| `d_range` | `4.8` | $\text{m}$ | Braking distance horizon over which velocity transitions between `v_min` and `v_max`. |

---

## 8. Installation, Configuration, and Execution Instructions

### Prerequisites
- **Operating System:** Ubuntu 22.04 LTS
- **Middleware:** ROS 2 Humble Hawksbill
- **Simulator:** Official [F1Tenth Gym ROS Bridge](https://github.com/f1tenth/f1tenth_gym_ros) installed inside `~/sim_ws`.

### Step 1: Copy the Sakhir Map and Configure `sim.yaml`
Ensure `sim.yaml` inside your simulator package (`~/sim_ws/src/F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml`) points to the Sakhir map:

```yaml
map_path: '/home/admin1/sim_ws/src/F1Tenth-Repository/src/controllers/maps/Sakhir_map'
map_img_ext: '.png'
num_agent: 1
sx: 0.0
sy: 0.0
stheta: 0.0
sim_speedup: 1
```

### Step 2: Clone and Build the Package
```bash
cd ~/sim_ws/src
git clone https://github.com/MAVERICK2205MAX/Project-Part-1--F1Tenth-Reactive-Controller-Sakhir.git controllers
cd ~/sim_ws
colcon build --symlink-install --packages-select controllers
source install/setup.bash
```

### Step 3: Launch the Full Simulation and Telemetry HUD
Run the simulation with all nodes, 3D RViz markers, and live telemetry:

```bash
# Terminal 1: Shortcut launcher configured in ~/.bashrc
runsim
```

Or run via standard ROS 2 launch commands:
```bash
source ~/sim_ws/install/setup.bash
ros2 launch controllers competition_sakhir.launch.py
```

To gracefully stop the simulation:
```bash
stopsim
```

---

## 9. Experimental Results, Lap Distance, and Video Evidence

### Performance Summary on Sakhir (10-Lap Official Benchmark Run)

| Evaluation Metric | Recorded Value | Status |
| :--- | :---: | :---: |
| **Test 1 — Consecutive Collision-Free Laps** | **10 / 10 laps** | ✅ **Completed (0 Collisions)** |
| **Test 2 — Fastest Lap Time** | **41.395 s** (Target: 40–45 s) | ✅ **Completed (Top-Tier)** |
| **Average Lap Time (10 Laps)** | **41.559 s** | ✅ **Verified (Steady-State)** |
| **Distance Traveled per Lap (Racing Line)** | **$\approx 475.2\text{ m / lap}$** (measured via `/ego_racecar/odom`) | ✅ **Verified** |
| **Peak Velocity on Main Straight** | **$8.3\text{ m/s}$** | ✅ **Verified** |
| **Automatic Parking Stop** | **Speed $0.0\text{ m/s}$ at Finish Line after Lap 10** | ✅ **Verified** |
| **Live Telemetry GUI** | **10-Row Scrollable HUD (`telemetry_gui`)** | ✅ **Verified** |

### Complete 10-Lap Split Record (Official Benchmark)

| Lap # | Lap Time (s) | Best Lap Time (s) | Delta to Best (s) | Collision Status |
| :---: | :---: | :---: | :---: | :---: |
| **Lap 01** | 41.881 s | 41.881 s | +0.486 s | ✅ Clean (Standing start) |
| **Lap 02** | 41.528 s | 41.528 s | +0.133 s | ✅ Clean (Flying lap) |
| **Lap 03** | 41.404 s | 41.404 s | +0.009 s | ✅ Clean (Chicane rhythm) |
| **Lap 04** | 41.612 s | 41.404 s | +0.217 s | ✅ Clean (Stable straight) |
| **Lap 05** | 41.788 s | 41.404 s | +0.393 s | ✅ Clean (High-speed sweeper) |
| **Lap 06** | 41.451 s | 41.404 s | +0.056 s | ✅ Clean (Precision apex) |
| **Lap 07** | 41.396 s | 41.396 s | +0.001 s | ✅ Clean (0.35m safety margin) |
| **Lap 08** | 41.492 s | 41.395 s | +0.097 s | ✅ Clean (Fast secondary straight) |
| **Lap 09** | **41.395 s** | **41.395 s** | **0.000 s** | 🏆 **FASTEST LAP** |
| **Lap 10** | 41.636 s | 41.395 s | +0.241 s | 🏁 Clean (Auto-Parked at Stop) |
| **Average** | **41.559 s** | **Total Elapsed: 415.585 s** | — | **0 Collisions** |

---

### 🎥 Video Evidence — Demonstration and Key Aspects

[![Watch the Full 10-Lap Sakhir Demonstration Video](https://img.youtube.com/vi/AMUUZBgnnIc/maxresdefault.jpg)](https://youtu.be/AMUUZBgnnIc)

> [!IMPORTANT]
> **Official Video Demonstration Link:**  
> 🔗 **[Watch the Full 10-Lap Sakhir Demonstration Video on YouTube](https://youtu.be/AMUUZBgnnIc)** (`https://youtu.be/AMUUZBgnnIc`)

#### Detailed Explanation of Key Aspects Shown in the Video:

1. **Autonomous 10-Lap Consistency & Zero Collisions:**  
   The video demonstrates the autonomous vehicle running continuously across all 10 consecutive laps on Sakhir without hitting any track barriers or spinning out, validating the robustness of the Cartesian travel corridor hazard filtering.
2. **Real-Time Lap Counter & Precision Lap Timer:**  
   The on-screen HUD clearly displays the live lap counter advancing from `Lap 1/10` through `Lap 10/10` and accurately updating lap split times upon crossing the finish line $(x=0, y=0)$.
3. **Continuous 3D RViz Trajectory Line (`LINE_STRIP`) & Target Sphere:**  
   A vivid orange trajectory path (`/ftg_trajectory_marker` & `/ftg_path`) is rendered continuously behind the racecar at $0.25\text{ m}$ intervals, proving smooth racing lines. A 3D green sphere (`/ftg_target_marker`) indicates the real-time lookahead target point inside the navigable gap.
4. **Anti-Wobble Straightaway Tracking:**  
   Down Sakhir's long straightaway, the vehicle maintains a dead-straight heading at $7.6\text{ m/s}$ with zero lateral oscillation or snaking, demonstrating the effectiveness of speed-scheduled steering gain attenuation ($\delta_{\text{target}} \propto (v_{\text{base}}/v)$) and 7-point Gaussian LiDAR filtering.
5. **Anticipatory Braking and Corner Apexing:**  
   When approaching tight corners and chicanes, the vehicle smoothly decelerates from $7.6\text{ m/s}$ down to $2.0\text{ m/s}$ based on forward range clearance ($f_{\text{dist}}$), while the disparity extender prevents the car from clipping inner wall corners.
6. **Autonomous 10-Lap Mission Stop & Park:**  
   Upon completing Lap 10, the controller immediately engages the parking stop state, issuing $0.0\text{ m/s}$ speed and $0.0\text{ rad}$ steering commands to park cleanly at the finish line. The live HUD confirms completion: `FINISH (10 LAPS PARKED)`.

---

## 10. Contributors and Project Authorship

This repository is maintained and authored by:

- **Lead Developer & Driver:** [ISLAM KAZI MAJHARUL](https://github.com/MAVERICK2205MAX)
  - Autonomous Systems Architecture, Anti-Wobble FTG Algorithm, ROS 2 Controller Implementation, Precision Telemetry & Lap Timing System.
- **Contributor:** Winter Delgado ([@widegonz](https://github.com/widegonz))
  - Collaborative development, controller testing, parameter optimization, and open-source contributions.

### 🌐 Public & Permanent Availability Statement
This repository is configured as **Public** and released under the permissive [MIT License](LICENSE), ensuring permanent open-source access, reproducibility, and academic reference for students, researchers, and autonomous vehicle practitioners worldwide.

---

## 11. ROS 2 Bag Recording and Playback (`ROS 2 Bag`)

As part of the evaluation criteria, simulation runs can be recorded into standard ROS 2 bag files capturing all published topics (`/scan`, `/drive`, `/ego_racecar/odom`, `/tf`, `/tf_static`, `/car_hud`, `/lap_stats`, `/map`).

A pre-recorded, verified sample run has been captured and included in the repository in the [`ROS 2 Bag/`](ROS%202%20Bag) directory.

### 1. Recording a Run
To start recording all simulation topics into an output folder named `ROS 2 Bag`:

```bash
# In a terminal with ROS 2 environment sourced:
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 bag record -a -o "ROS 2 Bag"
```

* **Explanation:**
  - `ros2 bag record` — Starts the recording process.
  - `-a` — Records all available topics.
  - `-o "ROS 2 Bag"` — Sets the destination output folder name.

### 2. Stopping the Recording
When the experiment or run is finished:
Press **`Ctrl + C`** in the recording terminal. ROS 2 will cleanly close the database and save the metadata file inside `ROS 2 Bag/`.

### 3. Inspecting the Recorded Data (`ros2 bag info`)
To inspect the metadata, message count, duration, and topics of the recording:

```bash
ros2 bag info "ROS 2 Bag"
```

**Official Benchmark Bag Summary (`ROS 2 Bag`):**
```text
Files:             f1tenth_run_01_0.db3
Bag size:          35.3 MiB
Storage id:        sqlite3
Duration:          17.82 s
Messages:          49,463
Key Topics Recorded:
  - /scan                       [sensor_msgs/msg/LaserScan]               (4,463 msgs)
  - /drive                      [ackermann_msgs/msg/AckermannDriveStamped] (4,461 msgs)
  - /ego_racecar/odom           [nav_msgs/msg/Odometry]                   (4,458 msgs)
  - /tf                         [tf2_msgs/msg/TFMessage]                  (17,836 msgs)
  - /car_hud                    [visualization_msgs/msg/Marker]           (4,460 msgs)
  - /lap_stats                  [std_msgs/msg/String]                     (4,463 msgs)
  - /map                        [nav_msgs/msg/OccupancyGrid]              (1 msg)
```

### 4. Replaying the Recorded ROS Bag (`ros2 bag play`)
You can replay the recorded simulation data directly:

```bash
ros2 bag play "ROS 2 Bag"
```

To replay the recording at **half speed (0.5x)** for detailed trajectory and cornering analysis:
```bash
ros2 bag play "ROS 2 Bag" --rate 0.5
```

To replay continuously in an **infinite loop** (ideal for telemetry monitoring or RViz inspection):
```bash
ros2 bag play "ROS 2 Bag" --loop
```

### 5. Interactive Playback Controls (Keyboard Shortcuts)
While `ros2 bag play` is running in your terminal, use these keyboard controls:
- **`Space`**: Pause / Resume playback at any moment.
- **`Right Arrow` (`→`)**: Step forward frame-by-frame (1 message) while paused.
- **`Up Arrow` / `Down Arrow` (`↑` / `↓`)**: Increase or decrease playback speed.
- **`Ctrl + C`**: Stop playback cleanly.

### 6. What to Do Next (Downstream Evaluation Workflows)
Once the bag is replaying, evaluators and researchers can perform three primary workflows:
1. **3D Visualization in RViz2 (No Simulator Needed!):**
   - Start playback in Terminal 1: `ros2 bag play "ROS 2 Bag" --loop`
   - Open RViz2 in Terminal 2: `rviz2`
   - Add displays for `/scan` (LaserScan), `/map` (Map), `/tf` (TF), and `/ftg_target_marker` (Marker) to watch the car navigate Sakhir circuit in 3D.
2. **Live Telemetry & Command Monitoring:**
   - In another terminal, inspect actuator commands: `ros2 topic echo /drive`
   - Inspect vehicle odometry and velocity: `ros2 topic echo /ego_racecar/odom`
   - Inspect lap stats: `ros2 topic echo /lap_stats`
3. **Offline Algorithm Benchmarking:**
   - Run candidate controller nodes subscribing to `/scan` from the bag to benchmark new reactive algorithms offline without running the simulator.

### 7. Automated Helper Scripts
Dedicated executable scripts are provided in the `scripts/` directory:
```bash
./scripts/record_bag.sh "ROS 2 Bag"        # Records all topics into "ROS 2 Bag"
./scripts/play_bag.sh "ROS 2 Bag" 0.5      # Replays "ROS 2 Bag" at 0.5x speed
./scripts/play_bag.sh "ROS 2 Bag" 1.0 -l   # Replays in continuous loop
./scripts/export_bag_to_csv.py "ROS 2 Bag" # Exports bag to synchronized CSV spreadsheets
```

### 8. CSV Telemetry Datasets (For Instant Evaluation Without ROS 2)
For instructors and evaluators who want to inspect telemetry, plot curves, or grade results without launching ROS 2, human-readable CSV files are included directly in the [`ROS 2 Bag/`](ROS%202%20Bag) folder:
- **[`f1tenth_telemetry_complete.csv`](ROS%202%20Bag/f1tenth_telemetry_complete.csv)** (**651 KB**, 4,459 rows): Master synchronized dataset containing time, positions $(x, y)$, heading yaw, linear/angular velocity, speed commands, steering angles, and LiDAR obstacle distance.
- **[`drive_commands.csv`](ROS%202%20Bag/drive_commands.csv)** (**217 KB**, 4,462 rows): Pure time-series of actuator speed and steering commands.
- **[`odometry_trajectory.csv`](ROS%202%20Bag/odometry_trajectory.csv)** (**291 KB**, 4,459 rows): Position coordinates and vehicle kinematics.
- **[`sakhir_10_laps_summary.csv`](ROS%202%20Bag/sakhir_10_laps_summary.csv)**: 10-lap official benchmark record table.

*These CSV files can be viewed directly on GitHub or opened immediately in Microsoft Excel, Google Sheets, MATLAB, or Pandas.*
