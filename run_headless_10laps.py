#!/usr/bin/env python3
"""
F1Tenth Competition Headless 10-Lap Verification Script
Track: Sakhir | Student: ISLAM KAZI MAJHARUL (Group 2)

Advanced Anti-Wobble & Racing Line Optimization:
- Speed-Scheduled Steering Gain Attenuation (prevents high-speed fishtailing)
- Time-normalized Slew Rate Limiter (consistent physical steering damping)
- Extended Lookahead (8.0m max range)
- Hazard Cone Constrained Safety Bubble (|theta| <= 58 deg, d < 1.8m)
- Disparity Extender with clean 1.0m hysteresis threshold
- Dynamic Speed Profiling: 2.2 m/s (hairpins) to 8.5 m/s (straights)
"""

import math
import os
import sys
import time
import gym
import numpy as np


class FollowTheGapController:
    """Optimized Anti-Wobble FTG Controller with Speed Gain Scheduling and Anticipatory Braking."""

    def __init__(self, bubble_radius=0.35, max_range=8.0, fov_deg=72.0, v_min=2.0, v_max=8.0):
        self.bubble_radius = bubble_radius
        self.max_range = max_range
        self.fov_deg = fov_deg
        self.v_min = v_min
        self.v_max = v_max
        self.prev_steer = 0.0
        self.prev_time = None

        self.hazard_cone_rad = math.radians(58.0)
        self.bubble_max_dist = 1.8
        self.disp_thresh = 1.0
        self.v_steer_base = 3.2
        self.v_steer_pow = 0.85
        self.max_steer_velocity = 2.8 # rad/s physical steering rate limit
        self.d_floor = 2.0
        self.d_range = 4.8

        # 7-point Gaussian kernel
        self.gauss_kernel = np.array([1, 3, 6, 7, 6, 3, 1], dtype=np.float32)
        self.gauss_kernel /= np.sum(self.gauss_kernel)

    def get_action(self, scan, current_time=None):
        num_beams = len(scan)
        angle_min = -2.356194490192345
        angle_inc = 4.71238898038469 / num_beams

        if current_time is None:
            dt = 0.01
        elif self.prev_time is None:
            dt = 0.01
            self.prev_time = current_time
        else:
            dt = max(0.001, min(0.05, current_time - self.prev_time))
            self.prev_time = current_time

        # 1. FOV restriction to [-fov_deg, +fov_deg]
        fov_rad = math.radians(self.fov_deg)
        min_idx = max(0, int((-fov_rad - angle_min) / angle_inc))
        max_idx = min(num_beams - 1, int((fov_rad - angle_min) / angle_inc))

        # 2. Clamping and Gaussian smoothing
        proc_ranges = np.array(scan[min_idx:max_idx + 1], dtype=np.float32)
        proc_ranges = np.nan_to_num(proc_ranges, nan=0.0, posinf=self.max_range, neginf=0.0)
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)
        proc_ranges = np.convolve(proc_ranges, self.gauss_kernel, mode='same')
        angles = angle_min + (min_idx + np.arange(len(proc_ranges))) * angle_inc

        # 3. Hazard cone constrained safety bubble
        cone_mask = np.abs(angles) <= self.hazard_cone_rad
        cone_indices = np.where(cone_mask)[0]
        if len(cone_indices) > 0:
            closest_local = cone_indices[int(np.argmin(proc_ranges[cone_mask]))]
            min_dist = float(proc_ranges[closest_local])
            if 0.05 < min_dist < self.bubble_max_dist:
                bubble_angle = math.atan2(self.bubble_radius, min_dist)
                bubble_beams = int(bubble_angle / angle_inc)
                b_start = max(0, closest_local - bubble_beams)
                b_end = min(len(proc_ranges), closest_local + bubble_beams + 1)
                proc_ranges[b_start:b_end] = 0.0

        # 4. Disparity extender (clean threshold with forward cone restriction)
        diff = np.abs(np.diff(proc_ranges))
        for d_idx in np.where(diff > self.disp_thresh)[0]:
            if abs(angles[d_idx]) <= math.radians(60.0):
                d1 = proc_ranges[d_idx]
                d2 = proc_ranges[d_idx + 1]
                if d1 < d2 and d1 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d1) / angle_inc)
                    proc_ranges[d_idx + 1:min(len(proc_ranges), d_idx + 1 + b_beams)] = 0.0
                elif d2 < d1 and d2 > 0.1:
                    b_beams = int(math.atan2(self.bubble_radius, d2) / angle_inc)
                    proc_ranges[max(0, d_idx - b_beams):d_idx + 1] = 0.0

        # 5. Find gaps
        free_mask = proc_ranges > 0.7
        gaps = []
        in_gap = False
        start = 0
        for i, free in enumerate(free_mask):
            if free and not in_gap:
                in_gap = True
                start = i
            elif not free and in_gap:
                in_gap = False
                gaps.append((start, i - 1))
        if in_gap:
            gaps.append((start, len(proc_ranges) - 1))

        if not gaps:
            return 0.0, self.v_min

        # 6. Max gap & plateau-median target selection
        max_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center = (max_gap[0] + max_gap[1]) // 2

        sub = proc_ranges[max_gap[0]:max_gap[1] + 1]
        max_d = np.max(sub)
        near_max = np.where(sub >= max(0.92 * max_d, max_d - 0.25))[0]
        deepest = max_gap[0] + int(np.median(near_max))

        # Dynamic center weighting: 60% center on straights, 50% in turns
        mid = len(proc_ranges) // 2
        fwd_dist = float(np.mean(proc_ranges[max(0, mid - 15):min(len(proc_ranges), mid + 15)]))
        center_w = 0.62 if fwd_dist > 4.0 else 0.52

        target_local_idx = int(center_w * gap_center + (1.0 - center_w) * deepest)
        raw_steer = angles[target_local_idx]

        # 7. Anticipatory Dynamic Curvature-Clearance Speed Profiling
        steer_abs = abs(self.prev_steer)
        steer_factor = max(0.0, 1.0 - (steer_abs / math.radians(20.0)))
        dist_factor = min(1.0, max(0.0, (fwd_dist - self.d_floor) / self.d_range))
        speed = float(np.clip(
            self.v_min + (self.v_max - self.v_min) * (0.50 * steer_factor + 0.50 * dist_factor),
            self.v_min,
            self.v_max
        ))

        # 8. Speed-Scheduled Steering Gain Attenuation (ELIMINATES HIGH-SPEED FISHTAILING)
        if speed > self.v_steer_base:
            gain = (self.v_steer_base / speed) ** self.v_steer_pow
            target_steer = raw_steer * max(0.35, min(1.0, gain))
        else:
            target_steer = raw_steer

        # 9. Low-Pass Filter & Physical Slew Rate Limiter
        alpha = 0.25
        filtered_steer = (1.0 - alpha) * self.prev_steer + alpha * target_steer
        
        max_d_steer = self.max_steer_velocity * dt
        steer_angle = float(np.clip(
            filtered_steer,
            self.prev_steer - max_d_steer,
            self.prev_steer + max_d_steer
        ))
        self.prev_steer = steer_angle

        return steer_angle, speed


def main():
    map_path = '/home/admin1/sim_ws/src/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sakhir_map'
    print("=" * 70)
    print("  F1TENTH SAKHIR COMPETITION - 10-LAP BENCHMARK VERIFICATION")
    print("  Student: ISLAM KAZI MAJHARUL  |  Group: Group 2  |  Track: Sakhir")
    print("=" * 70)

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    # Sakhir start pose: x=0.0, y=0.0, heading ~ 1.5252 rad
    obs, _, _, _ = env.reset(np.array([[0.0, 0.0, 1.5252]]))
    controller = FollowTheGapController(
        bubble_radius=0.35,
        max_range=8.0,
        fov_deg=72.0,
        v_min=2.0,
        v_max=8.0
    )

    target_laps = 10
    lap_count = 0
    lap_times = []
    lap_start_sim_time = 0.0
    sim_time = 0.0
    dt = 0.01
    prev_y = 0.0
    armed = False
    collision_occurred = False

    t_real_start = time.time()
    max_steps = 150000

    print("\nStarting simulation... Driving autonomously around Sakhir...")

    for step in range(max_steps):
        steer, speed = controller.get_action(obs['scans'][0], sim_time)
        obs, _, done, _ = env.step(np.array([[steer, speed]]))
        sim_time += dt
        x, y = obs['poses_x'][0], obs['poses_y'][0]

        # Arm trigger after car progresses around the track
        if y > 15.0 or x > 15.0 or (sim_time - lap_start_sim_time) > 15.0:
            armed = True

        # Detect finish line crossing (from y < 0 to y >= 0 with |x| < 3.0)
        if armed and prev_y < 0.0 and y >= 0.0 and abs(x) < 3.0:
            lap_time = sim_time - lap_start_sim_time
            lap_count += 1
            lap_times.append(lap_time)
            print(f"  🏁 Lap {lap_count:2d} / {target_laps} COMPLETED! "
                  f"Lap Time: {lap_time:6.2f} s | Total Elapsed: {sim_time:6.2f} s")
            lap_start_sim_time = sim_time
            armed = False
            if lap_count >= target_laps:
                break

        prev_y = y

        if obs['collisions'][0] > 0:
            print(f"\n❌ COLLISION DETECTED at step {step}, time {sim_time:.2f} s at ({x:.2f}, {y:.2f})!")
            collision_occurred = True
            break

    t_real_elapsed = time.time() - t_real_start

    print("\n" + "=" * 70)
    print("  RESULTS SUMMARY")
    print("=" * 70)
    print(f"  Completed Laps:     {lap_count} / {target_laps}")
    print(f"  Collisions:         {'NONE (0 collisions)' if not collision_occurred else 'YES (Failed)'}")
    print(f"  Simulation Wall Time: {t_real_elapsed:.2f} seconds")
    print("-" * 70)
    print("   Lap #  |   Lap Time (s)   |   Status")
    print("-" * 70)

    best_lap = min(lap_times) if lap_times else 0.0
    avg_lap = sum(lap_times) / len(lap_times) if lap_times else 0.0

    csv_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sakhir_lap_records.csv')
    with open(csv_path, 'w') as f:
        f.write("Lap_Number,Lap_Time_s,Best_Lap_s,Total_Time_s,Date_Time\n")
        cum_time = 0.0
        cur_best = float('inf')
        for i, t in enumerate(lap_times, 1):
            cum_time += t
            if t < cur_best:
                cur_best = t
            star = "  ⭐ FASTEST LAP!" if t == best_lap else ""
            print(f"   Lap {i:2d} |      {t:6.2f} s    |   Clean{star}")
            f.write(f"{i},{t:.3f},{cur_best:.3f},{cum_time:.3f},{time.strftime('%Y-%m-%d %H:%M:%S')}\n")

    print("-" * 70)
    print(f"  🏆 FASTEST LAP TIME: {best_lap:.2f} seconds")
    print(f"  ⏱️  AVERAGE LAP TIME: {avg_lap:.2f} seconds")
    print(f"  📁 Official log written to: {csv_path}")
    print("=" * 70)

    # Return exit code 0 if 10 clean laps completed, 1 if collision
    return 0 if (lap_count >= target_laps and not collision_occurred) else 1


if __name__ == '__main__':
    sys.exit(main())
