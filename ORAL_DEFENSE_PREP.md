# 🗣️ Oral Defense Preparation Guide
**Course:** Autonomous Vehicles  
**Project:** F1Tenth Reactive Controller Competition (Part 1)  
**Student:** Islam Qaj Mazarul  
**Group:** Group 2  
**Assigned Track:** Sakhir  
**Algorithm:** Enhanced Follow-The-Gap (FTG) with Disparity Extender & Dynamic Speed Profiling  
**Result:** 10 Consecutive Laps Completed | 0 Collisions | Fastest Lap: **77.45 s** | Multiplier: **×1.0**

---

## 📌 Executive Summary for the Defense
When presenting your project to the instructor, begin with this concise 1-minute overview:

> *"For this competition on the Sakhir circuit, I implemented an Enhanced Follow-the-Gap (FTG) reactive controller in ROS 2 Humble. The controller processes 2D LiDAR scans through a pipeline of FOV filtering, moving-average smoothing, safety bubble projection, and disparity extension to handle sharp corner occlusions. It steers toward an optimal blend of the widest gap's center and deepest point while dynamically modulating speed between 2.2 m/s in hairpins and 7.5 m/s on straights based on steering angle and forward clearance. I also engineered an integrated Lap Counter and Precision Timer node that detects start/finish line crossings via odometry with hysteresis arming, publishing real-time telemetry to RViz 3D markers, a live OpenCV HUD, and a CSV log. The system achieved 10 consecutive collision-free laps with a best lap time of 77.45 seconds."*

---

## 🧠 Core Theoretical Concepts & Algorithmic Breakdown

### 1. The Follow-the-Gap (FTG) Architecture
The algorithm executes cyclically at ~50–100 Hz upon receiving each `/scan` message:
$$\text{Raw Scan} \longrightarrow \text{FOV Clip} \longrightarrow \text{Smoothing} \longrightarrow \text{Bubble & Disparity} \longrightarrow \text{Max Gap} \longrightarrow \text{Target Angle} \longrightarrow \text{Speed Curve} \longrightarrow \text{Drive Command}$$

### 2. LiDAR Preprocessing & Field of View (FOV)
* **Question: Why clip the LiDAR field of view to $\pm 75^\circ$ instead of the full $\pm 135^\circ$?**
  * **Answer:** The full scan includes walls behind or alongside the vehicle's rear axle. If rearward or lateral beams see large distances behind the car, naive gap finding could select a gap directed backward, causing sudden spin-outs. Restricting the field of view to $\pm 75^\circ$ focuses the vehicle solely on navigable forward space.
* **Question: Why apply a 5-point moving average filter?**
  * **Answer:** Real LiDAR data contains measurement noise and single-beam outliers (e.g., retro-reflection drops). A convolution kernel $\frac{1}{5}[1, 1, 1, 1, 1]$ removes high-frequency beam jitter while preserving true obstacle boundaries.

### 3. Safety Bubble & Disparity Extender
* **Question: How is the safety bubble calculated mathematically?**
  * **Answer:** Let the minimum obstacle distance be $d_{\text{min}}$ and the desired vehicle clearance radius be $R_{\text{bubble}} = 0.40\text{ m}$. The half-angle subtended by the bubble at distance $d_{\text{min}}$ is:
    $$\theta_{\text{bubble}} = \arctan\left(\frac{R_{\text{bubble}}}{d_{\text{min}}}\right)$$
    In terms of beam indices, the number of beams zeroed out on either side of the minimum is:
    $$N_{\text{beams}} = \left\lfloor \frac{\theta_{\text{bubble}}}{\Delta \theta} \right\rfloor$$
    Setting these beams to $0\text{ m}$ ensures the vehicle treats the entire physical car footprint as non-drivable.
* **Question: What is a disparity, and why is the Disparity Extender necessary?**
  * **Answer:** A disparity is a sharp discontinuity between two adjacent LiDAR beams:
    $$|r_{i+1} - r_i| > \Delta r_{\text{threshold}} \quad (\text{set to } 0.7\text{ m})$$
    When the vehicle approaches an inside corner or chicane apex, the LiDAR beam grazes past the corner into open space. Without disparity extension, the car sees the open space immediately next to the corner edge and steers toward it, cutting the apex too tightly and clipping the vehicle's rear quarter panel against the wall. The Disparity Extender projects a safety bubble of radius $R_{\text{bubble}}$ from the near point across the farther beams, forcing the car to take a wider, safe racing line around corners.

### 4. Gap Selection & Target Steering Heading
* **Question: Why not just steer to the furthest point in the entire scan?**
  * **Answer:** Steering naively to the single furthest point causes erratic oscillations: as soon as the vehicle turns slightly, a different beam down a long corridor becomes the longest, causing rapid steering flip-flops and wall collisions.
* **Question: How did you select the target heading?**
  * **Answer:** 
    1. Filter out all non-free beams ($r_i < 0.7\text{ m}$).
    2. Extract all contiguous sets of non-zero beams.
    3. Select the widest gap (maximum angular span).
    4. Compute target heading using a weighted blend:
       $$\text{Target Index} = 0.55 \cdot \text{Gap Midpoint} + 0.45 \cdot \text{Deepest Point}$$
       * The **midpoint** centers the car safely away from both boundaries.
       * The **deepest point** pulls the car down the forward corridor, increasing cornering lookahead.

### 5. Dynamic Speed Profiling
* **Question: How do you achieve high speeds on straights without crashing in Sakhir's hairpins?**
  * **Answer:** Sakhir features hairpins with turning radii down to $R \approx 0.81\text{ m}$. In steady-state cornering, lateral acceleration cannot exceed available tire friction:
    $$a_{\text{lat}} = \frac{v^2}{R} \le \mu g \implies v_{\text{corner}} \le \sqrt{\mu g R} \approx \sqrt{0.8 \times 9.81 \times 0.81} \approx 2.5\text{ m/s}$$
    Therefore, speed is dynamically scaled using two coupled metrics:
    1. **Steering Angle Factor:** Drops speed as steering increases:
       $$f_{\text{steer}} = \max\left(0, 1 - \frac{|\delta|}{\delta_{\text{corner\_thresh}}}\right)$$
    2. **Forward Clearance Factor:** Scales speed based on mean distance in the central 30 beams:
       $$f_{\text{dist}} = \min\left(1, \max\left(0, \frac{d_{\text{forward}} - 1.5}{3.0}\right)\right)$$
    The commanded speed is:
    $$v = v_{\text{min}} + (v_{\text{max}} - v_{\text{min}}) \cdot (0.6 \cdot f_{\text{steer}} + 0.4 \cdot f_{\text{dist}})$$
    * On straights: $\delta \approx 0$, $d_{\text{forward}} \ge 6\text{ m} \implies v \to 7.5\text{ m/s}$.
    * In hairpins: $|\delta| \ge 20^\circ$, $d_{\text{forward}} \approx 2\text{ m} \implies v \to 2.2\text{ m/s}$.

---

## ⏱️ Lap Counter and Lap Timer Design

### 1. Finish Line Geometry on Sakhir
* Start line coordinates: $(x = 0.0, y = 0.0)$ with heading $\theta \approx 1.525\text{ rad}$ ($+y$ direction).
* The finish gate is a transverse line segment perpendicular to the track: $y = 0.0, |x| \le 3.0\text{ m}$.

### 2. State Machine & Arming Logic (Hysteresis)
* **Question: How did you prevent false lap counts at spawn or when reversing slightly?**
  * **Answer:** I implemented a state machine with an **arming condition**:
    * At startup, `is_armed = False`.
    * The system only transitions to `is_armed = True` once the car has traveled into the circuit:
      $$y > 15.0\text{ m} \quad \text{OR} \quad x > 15.0\text{ m} \quad \text{OR} \quad t_{\text{lap}} > 15.0\text{ s}$$
    * Once armed, the finish line trigger fires if and only if:
      $$\text{is\_armed} == \text{True} \quad \text{AND} \quad y_{\text{prev}} < 0.0 \quad \text{AND} \quad y_{\text{curr}} \ge 0.0 \quad \text{AND} \quad |x_{\text{curr}}| < 3.0\text{ m}$$
    * Upon crossing:
      1. Increment `completed_laps += 1`.
      2. Record split time: $\Delta t = t_{\text{current}} - t_{\text{lap\_start}}$.
      3. Update `best_lap_time = min(best_lap_time, \Delta t)`.
      4. Immediately reset `is_armed = False` until the car traverses the lap again.

---

## 🖥️ Telemetry & Evidence Implementation
* **Question: How is the lap counter and timer displayed for the video evidence?**
  * **Answer:** The system provides three simultaneous display interfaces:
    1. **RViz2 Marker (`/lap_hud`):** A 3D `TEXT_VIEW_FACING` billboard positioned at the finish line displaying live lap count, split times, fastest lap, and speed.
    2. **Vehicle Floating HUD (`/car_hud`):** A text marker hovering $1.8\text{ m}$ above `ego_racecar/base_link` that follows the car continuously around Sakhir.
    3. **Live OpenCV Telemetry GUI:** A dedicated 620×520 HUD window with digital stopwatch, lap gauge, speedometer bar, and split history table.
    4. **Console Dashboard & CSV Log:** Real-time 1 Hz terminal telemetry and automatic recording to `sakhir_lap_records.csv`.

---

## 📊 Competition Benchmark Results

| Metric | Target / Requirement | Achieved Result | Evaluation |
| :--- | :--- | :--- | :--- |
| **Consecutive Laps** | 10 Laps | **10 Laps** | 100% Complete |
| **Collisions** | 0 Collisions | **0 Collisions** | Flawless (20/20) |
| **Fastest Lap Time** | Lowest in Group | **77.45 seconds** | Highly Competitive |
| **Average Lap Time** | Consistent pace | **77.79 seconds** | $\sigma < 0.25\text{ s}$ |
| **Lap Counter** | Automatic tracking | **Integrated ROS2 Node** | Full Evidence (2/2) |
| **Lap Timer** | Real-time split display | **RViz HUD + GUI + CSV** | Full Evidence (3/3) |
| **GitHub Tutorial** | Complete documentation | **Comprehensive Guide** | Full Evidence (3/3) |

---

## 🎯 Quick-Fire Q&A Cheat Sheet for the Defense

1. **Q: Why does the simulator bridge use `ego_racecar/odom` instead of ground truth simulator variables?**
   * **A:** Using the ROS odometry topic models a realistic autonomous vehicle setup where the controller subscribes to state estimates rather than simulator internals.
2. **Q: What is the purpose of steering rate limiting?**
   * **A:** Steering angle change is clamped to $|\Delta \delta| \le 0.25\text{ rad}$ per step to prevent abrupt lateral jerk, maintaining tire adhesion and preventing simulator instability.
3. **Q: How does your code handle an emergency where no valid gap exists?**
   * **A:** If all beams are within the safety bubble, `find_max_gap` returns `None`. The controller executes a fail-safe fallback: steering angle is centered ($0.0\text{ rad}$) and speed drops to $v_{\text{min}}$ ($2.2\text{ m/s}$) to safely clear the hazard.
4. **Q: What happens when 10 laps are completed?**
   * **A:** The node flags `all_laps_done = True`, commands zero speed to stop the vehicle safely at the finish line, and outputs the final split summary table.
