# 🗣️ Oral Defense Preparation Guide
**Course:** Autonomous Vehicles  
**Project:** F1Tenth Reactive Controller Competition (Part 1)  
**Student:** ISLAM KAZI MAJHARUL  
**Group:** Group 2  
**Assigned Track:** Sakhir  
**Algorithm:** Enhanced Anti-Wobble Follow-The-Gap (FTG) with Disparity Extender & Dynamic Speed Profiling  
**Result:** 10 Consecutive Laps Completed | 0 Collisions | Fastest Lap: **71.51 s** | Multiplier: **×1.0**

---

## 📌 Executive Summary for the Defense
When presenting your project to the instructor, begin with this concise 1-minute overview:

> *"For this competition on the Sakhir circuit, I developed an Enhanced Anti-Wobble Follow-the-Gap (FTG) reactive controller in ROS 2 Humble. Through telemetry analysis, I resolved the classical FTG fishtailing problem by introducing a hazard-cone constrained safety bubble, 7-point Gaussian smoothing, plateau-median lookahead targeting, and an Exponential Moving Average (EMA) low-pass steering filter. The controller dynamically scales speed between 2.2 m/s in hairpins and 8.0 m/s on straights based on steering demand and forward clearance. I also integrated an automatic Lap Counter and Precision Timer node using odometry hysteresis arming, displaying live telemetry on RViz 3D text billboards, a 620×520 OpenCV GUI, and persistent CSV logs. The vehicle completed all 10 consecutive laps with 0 collisions and achieved a fastest lap time of 71.51 seconds."*

---

## 🧠 Core Theoretical Concepts & Algorithmic Breakdown

### 1. The Follow-the-Gap (FTG) Architecture
The algorithm executes cyclically at ~50–100 Hz upon receiving each `/scan` message:
$$\text{Raw Scan} \longrightarrow \text{Forward FOV & Gauss Filter} \longrightarrow \text{Hazard-Cone Bubble} \longrightarrow \text{Disparity Extender} \longrightarrow \text{Max Gap} \longrightarrow \text{Plateau Median Target} \longrightarrow \text{EMA Steering Filter} \longrightarrow \text{Speed Curve} \longrightarrow \text{Drive Command}$$

---

### 2. Anti-Wobble Stabilization & Trajectory Optimization (Key Technical Deep-Dive)

* **Question: Why do standard Follow-the-Gap controllers wobble and fishtail, and how did you mathematically fix it?**
  * **Answer:** Standard FTG suffers from high-frequency lateral oscillation due to three distinct phenomena that I identified and solved:
    1. **Side-Wall Bubble Alternation Paradox:** In a $2.2\text{ m}$ corridor, the closest points to the car in a $150^\circ$ scan are the parallel side walls ($|\theta| \approx 70^\circ\text{ to }75^\circ$) at distance $\approx 0.7\text{ m}$. If the car drifts by just $1\text{ mm}$, the global `argmin` jumps between the left and right side walls. In naive FTG, placing an unconstrained bubble of radius $0.40\text{ m}$ at $\pm 75^\circ$ zeroes out 120 forward beams on that side, creating an artificial gap on the opposite side and inducing an instantaneous $14^\circ$ steering jerk! In the next step, the vehicle nears the other wall, flipping the bubble and jerk in reverse.  
       * **Solution:** I introduced a **Hazard Cone Constraint**: the safety bubble is only applied if $|\theta_i| \le 58^\circ$ and $d_i < 1.80\text{ m}$. Parallel side walls are preserved as track boundaries, while true frontal collision risks are securely cleared.
    2. **Discrete `argmax` Jitter on Straights:** On open straights, ranges form a flat plateau where multiple beams hit the $6.0\text{ m}$ ceiling. A standard `np.argmax()` returns the first index of the plateau (the far left beam), pulling the car toward the wall until raycast discretization causes it to jump.  
       * **Solution:** I implemented **Plateau-Median Centering**: the algorithm finds all beams within the upper $8\%$ of maximum depth in the gap and computes their **median**:
         $$i_{\text{deepest}} = \operatorname{median}\left(\{i \in \text{gap} \mid r_i \ge \max(0.92 \cdot r_{\text{max}}, r_{\text{max}} - 0.25)\} \right)$$
         Targeting $0.55 \cdot \text{center} + 0.45 \cdot i_{\text{deepest}}$ guarantees perfect straight-line tracking.
    3. **Undamped Steering Rate Limits:** A simple rate limiter `clip(steer, prev - d, prev + d)` creates a triangular limit-cycle oscillation.  
       * **Solution:** I implemented an **Exponential Moving Average (EMA) Low-Pass Filter**:
         $$\delta_k = (1 - \alpha) \cdot \delta_{k-1} + \alpha \cdot \delta_{\text{raw}} \quad (\alpha = 0.28)$$
         with a slew rate clamp $\Delta \delta_{\text{max}} = 0.15\text{ rad/step}$.
    * **Empirical Validation:** In empirical tests over 1200 simulation steps, significant steering reversals dropped from **455 down to only 68**, and the car completed all laps without a trace of wobbling.

---

### 3. LiDAR Preprocessing & Field of View (FOV)
* **Question: Why clip the LiDAR field of view to $\pm 72^\circ$ instead of the full $\pm 135^\circ$?**
  * **Answer:** The Hokuyo LiDAR publishes 1080 beams across $270^\circ$. Beams pointing rearward or directly lateral ($|\theta| > 72^\circ$) see empty space behind the vehicle or sideways along open runoffs. If naive gap extraction considers these beams, it can command a sharp turn into a non-navigable wall or spin the vehicle. Restricting FOV to $[ -72^\circ, +72^\circ ]$ focuses the vehicle solely on navigable forward track geometry.
* **Question: Why use a 7-point Gaussian filter instead of a boxcar moving average?**
  * **Answer:** A boxcar filter ($K = \frac{1}{N}[1, \dots, 1]$) has high side lobes in the frequency domain, causing ringing artifacts near sharp obstacle boundaries. A 7-point Gaussian kernel ($K = \frac{1}{27}[1, 3, 6, 7, 6, 3, 1]$) acts as an optimal Gaussian low-pass filter: it attenuates high-frequency beam noise and raycast discretization while preserving sharp edge locations without phase shift.

---

### 4. Safety Bubble & Disparity Extender
* **Question: How is the safety bubble calculated mathematically?**
  * **Answer:** Let the minimum obstacle distance be $d_{\text{min}}$ and the desired vehicle clearance radius be $R_{\text{bubble}} = 0.35\text{ m}$. The half-angle subtended by the bubble at distance $d_{\text{min}}$ is:
    $$\theta_{\text{bubble}} = \arctan\left(\frac{R_{\text{bubble}}}{d_{\text{min}}}\right)$$
    In terms of beam indices, the number of beams zeroed out on either side of the obstacle is:
    $$N_{\text{beams}} = \left\lfloor \frac{\theta_{\text{bubble}}}{\Delta \theta} \right\rfloor$$
    Setting these beams to $0\text{ m}$ forces the vehicle to treat the physical car footprint as impassable.
* **Question: What is a disparity, and why is the Disparity Extender necessary?**
  * **Answer:** A disparity is a sharp radial distance jump between two adjacent LiDAR beams:
    $$|r_{i+1} - r_i| > \Delta r_{\text{threshold}} \quad (\text{threshold } = 0.7\text{ m})$$
    When the vehicle approaches an inside corner or chicane apex, the LiDAR beam grazes past the corner into open space. Without disparity extension, the car sees the open space immediately next to the corner edge and steers toward it, cutting the apex too tightly and clipping the vehicle's rear quarter panel against the wall. The Disparity Extender projects a safety bubble of radius $R_{\text{bubble}}$ from the near point across the farther beams, forcing the car to take a wider, safe racing line around corners.

---

### 5. Dynamic Speed Profiling & Vehicle Dynamics
* **Question: How do you achieve high speeds on straights without crashing in Sakhir's hairpins?**
  * **Answer:** Sakhir features hairpins with turning radii down to $R \approx 0.81\text{ m}$. In steady-state cornering, lateral acceleration cannot exceed available tire friction:
    $$a_{\text{lat}} = \frac{v^2}{R} \le \mu g \implies v_{\text{corner}} \le \sqrt{\mu g R} \approx \sqrt{0.8 \times 9.81 \times 0.81} \approx 2.5\text{ m/s}$$
    Therefore, speed is dynamically scaled using two coupled metrics:
    1. **Steering Angle Factor:** Drops speed as steering increases:
       $$f_{\text{steer}} = \max\left(0.0, 1.0 - \frac{|\delta|}{\delta_{\text{corner\_thresh}}}\right) \quad (\delta_{\text{corner\_thresh}} = 24^\circ)$$
    2. **Forward Clearance Factor:** Scales speed based on mean distance in the central 30 beams:
       $$f_{\text{dist}} = \min\left(1.0, \max\left(0.0, \frac{d_{\text{forward}} - 1.4}{3.0}\right)\right)$$
    The commanded speed is:
    $$v = v_{\text{min}} + (v_{\text{max}} - v_{\text{min}}) \cdot (0.60 \cdot f_{\text{steer}} + 0.40 \cdot f_{\text{dist}})$$
    * On straights: $\delta \approx 0$, $d_{\text{forward}} \ge 6\text{ m} \implies v \to 8.0\text{ m/s}$.
    * In hairpins: $|\delta| \ge 20^\circ$, $d_{\text{forward}} \approx 2\text{ m} \implies v \to 2.2\text{ m/s}$.
    * This improved our fastest lap from **77.45 seconds to 71.51 seconds** (a nearly 6-second improvement).

---

## ⏱️ Lap Counter and Lap Timer Design

### 1. Finish Line Geometry on Sakhir
* Start line coordinates: $(x = 0.0, y = 0.0)$ with heading $\theta \approx 1.525\text{ rad}$ ($+y$ direction).
* The finish gate is a transverse line segment perpendicular to the track: $y = 0.0, |x| \le 3.0\text{ m}$.

### 2. State Machine & Arming Logic (Hysteresis)
* **Question: How did you prevent false lap counts at spawn or when reversing slightly?**
  * **Answer:** I implemented a state machine with a **strict hysteresis arming condition**:
    * At startup, `is_armed = False`.
    * The system only transitions to `is_armed = True` once the car has traveled $> 25.0\text{ m}$ away into the circuit (or $t_{\text{lap}} \ge 15.0\text{ s}$).
    * Once armed, the finish line trigger fires if and only if:
      $$\text{is\_armed} == \text{True} \quad \text{AND} \quad y_{\text{prev}} < 0.0 \quad \text{AND} \quad y_{\text{curr}} \ge 0.0 \quad \text{AND} \quad |x_{\text{curr}}| < 3.0\text{ m}$$
    * Upon crossing:
      1. Increment `completed_laps += 1`.
      2. Record split time: $\Delta t = t_{\text{current}} - t_{\text{lap\_start}}$.
      3. Update `best_lap_time = min(best_lap_time, \Delta t)`.
      4. Immediately reset `is_armed = False` until the car completes the next circuit.

---

## 🖥️ Telemetry & Evidence Implementation
* **Question: How is the lap counter and timer displayed for the video evidence?**
  * **Answer:** The system provides four simultaneous display interfaces:
    1. **RViz2 Marker (`/lap_hud`):** A 3D `TEXT_VIEW_FACING` billboard positioned at the finish line displaying live lap count, split times, fastest lap, and speed.
    2. **Vehicle Floating HUD (`/car_hud`):** A text marker hovering $1.2\text{ m}$ directly above `ego_racecar/base_link` that follows the car continuously around Sakhir.
    3. **Live OpenCV Telemetry GUI:** A dedicated 620×520 HUD window with digital stopwatch, lap gauge, speedometer bar, and split history table.
    4. **Console Dashboard & CSV Log:** Real-time 1 Hz terminal telemetry and automatic recording to `sakhir_lap_records.csv`.
