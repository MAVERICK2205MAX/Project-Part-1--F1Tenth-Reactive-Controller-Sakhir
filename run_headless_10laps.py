#!/usr/bin/env python3
"""
F1Tenth Competition Headless 10-Lap Verification Script
Track: Sakhir | Student: Islam Qaj Mazarul (Group 2)

Runs the Follow-the-Gap reactive controller for 10 complete consecutive laps
in fast-forward simulation, verifying:
1. 0 Collisions across 10 consecutive laps.
2. Exact lap times, average lap time, and fastest lap time.
3. Automatically exports official lap split logs to CSV.
"""

import math
import os
import sys
import time
import gym
import numpy as np


class FollowTheGapController:
    """Follow-The-Gap reactive controller with Disparity Extender."""

    def __init__(self, bubble_radius=0.40, max_range=6.0, fov_deg=75.0, v_min=2.2, v_max=7.5):
        self.bubble_radius = bubble_radius
        self.max_range = max_range
        self.fov_deg = fov_deg
        self.v_min = v_min
        self.v_max = v_max
        self.prev_steer = 0.0

    def get_action(self, scan):
        num_beams = len(scan)
        angle_min = -2.356194490192345
        angle_inc = 4.71238898038469 / num_beams

        # 1. FOV restriction to [-fov_deg, +fov_deg]
        fov_rad = math.radians(self.fov_deg)
        min_idx = max(0, int((-fov_rad - angle_min) / angle_inc))
        max_idx = min(num_beams - 1, int((fov_rad - angle_min) / angle_inc))

        # 2. Clamping and moving average smoothing
        proc_ranges = np.array(scan[min_idx:max_idx + 1], dtype=np.float32)
        proc_ranges = np.clip(proc_ranges, 0.0, self.max_range)
        kernel = np.ones(5) / 5.0
        proc_ranges = np.convolve(proc_ranges, kernel, mode='same')

        # 3. Closest point and safety bubble
        closest_idx = np.argmin(proc_ranges)
        min_dist = proc_ranges[closest_idx]
        if min_dist > 0.05:
            bubble_angle = math.atan2(self.bubble_radius, min_dist)
            bubble_beams = int(bubble_angle / angle_inc)
            b_start = max(0, closest_idx - bubble_beams)
            b_end = min(len(proc_ranges), closest_idx + bubble_beams + 1)
            proc_ranges[b_start:b_end] = 0.0

        # 4. Disparity extender (occlusion shadows)
        diff = np.abs(np.diff(proc_ranges))
        disparities = np.where(diff > 0.7)[0]
        for d_idx in disparities:
            d1 = proc_ranges[d_idx]
            d2 = proc_ranges[d_idx + 1]
            if d1 < d2 and d1 > 0.1:
                b_ang = math.atan2(self.bubble_radius, d1)
                b_beams = int(b_ang / angle_inc)
                proc_ranges[d_idx + 1:min(len(proc_ranges), d_idx + 1 + b_beams)] = 0.0
            elif d2 < d1 and d2 > 0.1:
                b_ang = math.atan2(self.bubble_radius, d2)
                b_beams = int(b_ang / angle_inc)
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

        # 6. Max gap & target selection
        max_gap = max(gaps, key=lambda g: g[1] - g[0])
        gap_center = (max_gap[0] + max_gap[1]) // 2
        deepest_in_gap = max_gap[0] + np.argmax(proc_ranges[max_gap[0]:max_gap[1] + 1])

        target_local_idx = int(0.55 * gap_center + 0.45 * deepest_in_gap)
        target_global_idx = min_idx + target_local_idx
        steer_angle = angle_min + target_global_idx * angle_inc

        # 7. Steering rate limiter
        max_d_steer = 0.25
        steer_angle = np.clip(steer_angle, self.prev_steer - max_d_steer, self.prev_steer + max_d_steer)
        self.prev_steer = steer_angle

        # 8. Dynamic speed profiling
        steer_abs = abs(steer_angle)
        mid = len(proc_ranges) // 2
        fwd_dist = np.mean(proc_ranges[max(0, mid - 15):min(len(proc_ranges), mid + 15)])

        steer_factor = max(0.0, 1.0 - (steer_abs / math.radians(25.0)))
        dist_factor = min(1.0, max(0.0, (fwd_dist - 1.5) / 3.0))
        speed = self.v_min + (self.v_max - self.v_min) * (0.6 * steer_factor + 0.4 * dist_factor)
        speed = max(self.v_min, min(self.v_max, speed))

        return float(steer_angle), float(speed)


def main():
    map_path = '/home/admin1/sim_ws/src/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sakhir_map'
    print("=" * 70)
    print("  F1TENTH SAKHIR COMPETITION - 10-LAP BENCHMARK VERIFICATION")
    print("  Student: Islam Qaj Mazarul  |  Group: Group 2  |  Track: Sakhir")
    print("=" * 70)

    env = gym.make('f110_gym:f110-v0', map=map_path, map_ext='.png', num_agents=1)
    # Sakhir start pose: x=0.0, y=0.0, heading ~ 1.5252 rad
    obs, _, _, _ = env.reset(np.array([[0.0, 0.0, 1.5252]]))
    controller = FollowTheGapController(bubble_radius=0.40, max_range=6.0, fov_deg=75.0, v_min=2.2, v_max=7.5)

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
        steer, speed = controller.get_action(obs['scans'][0])
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

    if lap_times:
        best_lap = min(lap_times)
        avg_lap = sum(lap_times) / len(lap_times)
        print("-" * 70)
        print("   Lap #  |   Lap Time (s)   |   Status")
        print("-" * 70)
        for i, lt in enumerate(lap_times, 1):
            star = "  ⭐ FASTEST LAP!" if lt == best_lap else ""
            print(f"   Lap {i:2d} |    {lt:7.2f} s    |   Clean{star}")
        print("-" * 70)
        print(f"  🏆 FASTEST LAP TIME: {best_lap:.2f} seconds")
        print(f"  ⏱️  AVERAGE LAP TIME: {avg_lap:.2f} seconds")

        # Save to CSV
        csv_file = "sakhir_lap_records.csv"
        with open(csv_file, "w") as f:
            f.write("Lap_Number,Lap_Time_s,Best_Lap_s,Status\n")
            b_so_far = float('inf')
            for i, lt in enumerate(lap_times, 1):
                if lt < b_so_far:
                    b_so_far = lt
                f.write(f"{i},{lt:.3f},{b_so_far:.3f},Clean\n")
        print(f"  📁 Official log written to: {os.path.abspath(csv_file)}")

    print("=" * 70 + "\n")


if __name__ == '__main__':
    main()
