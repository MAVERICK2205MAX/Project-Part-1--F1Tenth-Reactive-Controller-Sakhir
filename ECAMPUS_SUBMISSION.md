# 🎓 Autonomous Vehicles — Project Part 1: Reactive Controller Competition
## Official E-Campus / LMS Submission Document

---

### 📋 Student & Course Metadata
- **Course:** Autonomous Vehicles (II PAO 2026)
- **Evaluation:** First Midterm Project (Part 1) — Reactive Controllers Competition
- **Student / Driver:** ISLAM KAZI MAJHARUL
- **Group:** Group 2
- **Assigned Track:** Sakhir Circuit
- **Contributor:** Winter Delgado (@widegonz)

---

### 🔗 Submission Deliverables
- **GitHub Repository (Public & Open Source):**  
  👉 [https://github.com/MAVERICK2205MAX/Project-Part-1--F1Tenth-Reactive-Controller-Sakhir](https://github.com/MAVERICK2205MAX/Project-Part-1--F1Tenth-Reactive-Controller-Sakhir)
- **YouTube Video Demonstration (Full 10-Lap Validation):**  
  👉 [https://youtu.be/AMUUZBgnnIc](https://youtu.be/AMUUZBgnnIc)

---

### 🏆 Grading Rubric Summary & Benchmark Results

| Rubric Criteria | Required Metric | Achieved Value | Status |
| :--- | :---: | :---: | :---: |
| **Challenge 1 — 10 Consecutive Laps** | 10 laps without collision | **10 / 10 laps (0 Collisions)** | ✅ **100% Met** |
| **Challenge 2 — Lap Time Window** | 40–45 seconds / lap | **41.395 s (Fastest) \| 41.559 s (Avg)** | ✅ **100% Met** |
| **Lap Counter Implementation (+2 pts)** | Autonomous finish line tracking | **Spatial hysteresis crossing detection** | ✅ **100% Met** |
| **Lap Timer Implementation (+3 pts)** | Individual splits & best lap | **High-precision ROS 2 timer & HUD** | ✅ **100% Met** |
| **Live Telemetry & 3D Trajectory** | Real-time monitoring | **OpenCV HUD + RViz LINE_STRIP** | ✅ **100% Met** |
| **Autonomous Mission Stop** | Park car after Lap 10 | **50 Hz active parking brake (0.0 m/s)** | ✅ **100% Met** |
| **GitHub Technical Documentation (+3 pts)** | Math, tuning & architecture | **Exhaustive README.md & Defense Guide** | ✅ **100% Met** |
| **Video Evidence (+2 pts)** | Verified screencast | **7:14 Full 10-lap YouTube Video** | ✅ **100% Met** |

---

### ⏱️ Official 10-Lap Split Times (Sakhir Circuit)

| Lap # | Split Time (s) | Best Lap Time (s) | Delta to Best (s) | Collision Status | Vehicle State |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Lap 01** | **41.881 s** | 41.881 s | +0.486 s | ✅ Clean (0 Col) | Standing Start |
| **Lap 02** | **41.528 s** | 41.528 s | +0.133 s | ✅ Clean (0 Col) | Flying Lap |
| **Lap 03** | **41.404 s** | 41.404 s | +0.009 s | ✅ Clean (0 Col) | Chicane Rhythm |
| **Lap 04** | **41.612 s** | 41.404 s | +0.217 s | ✅ Clean (0 Col) | Stable Straight |
| **Lap 05** | **41.788 s** | 41.404 s | +0.393 s | ✅ Clean (0 Col) | High-Speed Sweeper |
| **Lap 06** | **41.451 s** | 41.404 s | +0.056 s | ✅ Clean (0 Col) | Technical Infield |
| **Lap 07** | **41.396 s** | 41.396 s | +0.001 s | ✅ Clean (0 Col) | Corridor Centering |
| **Lap 08** | **41.492 s** | 41.395 s | +0.097 s | ✅ Clean (0 Col) | Fast Secondary Straight |
| **Lap 09** | **41.395 s** | **41.395 s** | **0.000 s** | 🏆 **Clean (0 Col)** | **FASTEST LAP RECORD** |
| **Lap 10** | **41.636 s** | 41.395 s | +0.241 s | 🏁 **Clean (0 Col)** | **AUTO-PARKED AT STOP (0.0 m/s)** |
| **Total / Avg** | **Total: 415.585 s** | **Best: 41.395 s** | **Avg: 41.559 s** | **0 COLLISIONS** | **MISSION COMPLETE** |

---

### 🧠 Algorithmic Innovations & Methodology
1. **Speed-Scheduled Steering Gain Attenuation:** Resolves classical FTG fishtailing ($a_y \approx \frac{v^2}{L}\delta$) by dynamically scaling steering gain inverse to velocity ($\delta_{\text{target}} \propto (v_{\text{base}}/v)$), ensuring laser-straight straightaways at $8.3\text{ m/s}$ and high maneuverability at hairpins.
2. **Cartesian Travel Corridor Hazard Filtering:** Projects raw LiDAR points to vehicle coordinates ($x = r\cos\theta, y = r\sin\theta$) and applies safety bubbles only to obstacles directly in the car's corridor ($|y| \le 0.45\text{ m}$), eliminating wall bubble gap-flipping.
3. **Disparity Extender:** Detects sudden depth jumps ($>1.0\text{ m}$) between adjacent beams and inflates obstacle edges by half the car width ($0.40\text{ m}$), preventing the vehicle from clipping inner apexes.
4. **Anticipatory Braking Profile:** Dynamically modulates longitudinal speed between $2.8\text{ m/s}$ and $8.3\text{ m/s}$ based on forward range clearance and steering curvature.
5. **Autonomous Mission Parking Stop:** Immediately upon detecting completion of Lap 10, the node engages an active 50 Hz parking brake timer that commands $0.0\text{ m/s}$ and stops the vehicle at the finish line.

---

### 💻 Quick Verification Command for the Instructor
To run the full simulation with all nodes, RViz markers, and live telemetry HUD:

```bash
cd ~/sim_ws/src
git clone https://github.com/MAVERICK2205MAX/Project-Part-1--F1Tenth-Reactive-Controller-Sakhir.git controllers
cd ~/sim_ws && colcon build --symlink-install --packages-select controllers
source install/setup.bash
ros2 launch controllers competition_sakhir.launch.py
```
*(Or simply run `runsim` in terminal)*.

### 💾 ROS 2 Bag Replay Verification (`f1tenth_run_01`)
A pre-recorded, verified ROS 2 bag is provided directly in `f1tenth_run_01/`:
```bash
# Inspect the recorded bag (49,463 messages, 35.3 MiB):
ros2 bag info f1tenth_run_01

# Replay at normal speed:
ros2 bag play f1tenth_run_01

# Replay at half speed (0.5x):
ros2 bag play f1tenth_run_01 --rate 0.5
```
