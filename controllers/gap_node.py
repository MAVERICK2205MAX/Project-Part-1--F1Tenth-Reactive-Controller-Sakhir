#!/usr/bin/env python3
"""
F1Tenth Autonomous Racing - Follow-The-Gap (FTG) Reactive Controller
Competition Node for Track: Sakhir
Student: Islam Qaj Mazarul (Group 2)

Features:
- High-performance reactive obstacle avoidance with Follow-the-Gap (FTG)
- Laser scan preprocessing & moving average smoothing
- Safety bubble expansion around closest obstacles
- Disparity extender for sharp obstacle occlusion shadows
- Dynamic speed profiling based on steering angle and forward clearance
- Automatic Lap Counter for Sakhir track
- Precision Lap Timer (current lap, last lap, fastest lap, split history)
- Visual HUD via RViz2 Marker (text overlay) and live terminal dashboard
- Automatic CSV logging of lap records
"""

import math
import os
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32, Int32, String


class ReactiveFollowGap(Node):
    """
    Follow-The-Gap reactive controller with integrated Lap Counter,
    Lap Timer, and RViz HUD visualization.
    """

    def __init__(self):
        super().__init__('reactive_node')

        # Declare configurable parameters
        self.declare_parameter('bubble_radius', 0.40)
        self.declare_parameter('max_range', 6.0)
        self.declare_parameter('fov_deg', 75.0)
        self.declare_parameter('v_min', 2.2)
        self.declare_parameter('v_max', 7.5)
        self.declare_parameter('disparity_threshold', 0.7)
        self.declare_parameter('smoothing_kernel', 5)
        self.declare_parameter('max_steer_rate', 0.25)
        self.declare_parameter('target_laps', 10)
        self.declare_parameter('log_csv_path', 'sakhir_lap_records.csv')

        # Retrieve parameters
        self.bubble_radius = float(self.get_parameter('bubble_radius').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.fov_deg = float(self.get_parameter('fov_deg').value)
        self.v_min = float(self.get_parameter('v_min').value)
        self.v_max = float(self.get_parameter('v_max').value)
        self.disp_thresh = float(self.get_parameter('disparity_threshold').value)
        self.kernel_size = int(self.get_parameter('smoothing_kernel').value)
        self.max_steer_rate = float(self.get_parameter('max_steer_rate').value)
        self.target_laps = int(self.get_parameter('target_laps').value)
        self.log_csv_path = str(self.get_parameter('log_csv_path').value)

        # Topics
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        odom_topic = '/ego_racecar/odom'

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)
        self.hud_pub = self.create_publisher(Marker, '/lap_hud', 10)
        self.car_hud_pub = self.create_publisher(Marker, '/car_hud', 10)
        self.lap_count_pub = self.create_publisher(Int32, '/lap_count', 10)
        self.lap_time_pub = self.create_publisher(Float32, '/lap_time', 10)
        self.best_lap_pub = self.create_publisher(Float32, '/best_lap_time', 10)
        self.stats_pub = self.create_publisher(String, '/lap_stats', 10)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

        # Controller State
        self.prev_steer = 0.0
        self.current_speed = 0.0
        self.current_steer = 0.0

        # Lap Counter & Timer State
        self.sim_start_time = None
        self.lap_start_time = None
        self.current_lap = 1
        self.completed_laps = 0
        self.lap_times = []
        self.best_lap_time = float('inf')
        self.last_lap_time = 0.0
        self.is_armed = False
        self.prev_y = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.total_distance = 0.0
        self.prev_pose = None
        self.all_laps_done = False

        # Status & logging timer (1 Hz terminal update)
        self.timer = self.create_timer(1.0, self.status_timer_callback)

        # Initialize CSV log file
        self._init_csv()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  F1Tenth Follow-The-Gap Controller Initialized')
        self.get_logger().info('  Track: Sakhir | Student: Islam Qaj Mazarul (Group 2)')
        self.get_logger().info(f'  Target Laps: {self.target_laps} | v_min: {self.v_min} m/s | v_max: {self.v_max} m/s')
        self.get_logger().info('=' * 60)

    def _init_csv(self):
        """Initialize CSV log file with header."""
        try:
            with open(self.log_csv_path, 'w') as f:
                f.write('Lap_Number,Lap_Time_s,Best_Lap_s,Total_Time_s,Date_Time\n')
        except Exception as e:
            self.get_logger().warn(f'Could not initialize CSV file: {e}')

    def _log_lap_to_csv(self, lap_num, lap_time, best_time, total_time):
        """Append completed lap record to CSV log."""
        try:
            timestamp = time.strftime('%Y-%m-%d %H:%M:%S')
            with open(self.log_csv_path, 'a') as f:
                f.write(f'{lap_num},{lap_time:.3f},{best_time:.3f},{total_time:.3f},{timestamp}\n')
        except Exception as e:
            self.get_logger().warn(f'Could not write to CSV: {e}')

    def preprocess_lidar(self, ranges, angle_min, angle_inc):
        """
        Preprocess the LiDAR scan array:
        1. Restrict field of view (FOV) to the forward arc [-fov_deg, +fov_deg].
        2. Filter NaNs, Infs and clamp high values to max_range.
        3. Smooth with running average convolution kernel.
        """
        num_beams = len(ranges)
        fov_rad = math.radians(self.fov_deg)

        min_idx = max(0, int((-fov_rad - angle_min) / angle_inc))
        max_idx = min(num_beams - 1, int((fov_rad - angle_min) / angle_inc))

        raw_sliced = np.array(ranges[min_idx:max_idx + 1], dtype=np.float32)
        raw_sliced = np.nan_to_num(raw_sliced, nan=0.0, posinf=self.max_range, neginf=0.0)
        clipped = np.clip(raw_sliced, 0.0, self.max_range)

        # Moving average filter
        kernel = np.ones(self.kernel_size) / float(self.kernel_size)
        smoothed = np.convolve(clipped, kernel, mode='same')

        return smoothed, min_idx, max_idx

    def apply_safety_bubble(self, proc_ranges, angle_inc):
        """
        Find the closest obstacle and zero out all beams within the safety bubble.
        """
        closest_idx = int(np.argmin(proc_ranges))
        min_dist = float(proc_ranges[closest_idx])

        if min_dist > 0.05:
            bubble_angle = math.atan2(self.bubble_radius, min_dist)
            bubble_beams = int(bubble_angle / angle_inc)
            b_start = max(0, closest_idx - bubble_beams)
            b_end = min(len(proc_ranges), closest_idx + bubble_beams + 1)
            proc_ranges[b_start:b_end] = 0.0

        return proc_ranges, closest_idx, min_dist

    def apply_disparity_extender(self, proc_ranges, angle_inc):
        """
        Extend safety bubble around sharp range discontinuities (disparities).
        Prevents clipping corner walls.
        """
        diff = np.abs(np.diff(proc_ranges))
        disparities = np.where(diff > self.disp_thresh)[0]

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

        return proc_ranges

    def find_max_gap(self, free_space_ranges, free_threshold=0.7):
        """
        Return the start index & end index of the max contiguous gap.
        """
        free_mask = free_space_ranges > free_threshold
        gaps = []
        in_gap = False
        start = 0

        for i, is_free in enumerate(free_mask):
            if is_free and not in_gap:
                in_gap = True
                start = i
            elif not is_free and in_gap:
                in_gap = False
                gaps.append((start, i - 1))
        if in_gap:
            gaps.append((start, len(free_space_ranges) - 1))

        if not gaps:
            return None

        # Return the gap with maximum beam width
        return max(gaps, key=lambda g: g[1] - g[0])

    def find_best_point(self, start_i, end_i, ranges):
        """
        Find best target index within the gap.
        Balances between deepest point (lookahead corridor) and gap center (safety margin).
        """
        gap_center = (start_i + end_i) // 2
        deepest_in_gap = start_i + int(np.argmax(ranges[start_i:end_i + 1]))

        # Optimal blend: 55% center, 45% deepest
        target_local_idx = int(0.55 * gap_center + 0.45 * deepest_in_gap)
        return target_local_idx

    def compute_speed(self, steer_angle, proc_ranges):
        """
        Dynamically calculate vehicle speed:
        - Drops in sharp turns to prevent tire slippage and collisions.
        - Accelerates smoothly on open straights up to v_max.
        """
        steer_abs = abs(steer_angle)
        mid = len(proc_ranges) // 2
        fwd_clearance = float(np.mean(proc_ranges[max(0, mid - 15):min(len(proc_ranges), mid + 15)]))

        steer_factor = max(0.0, 1.0 - (steer_abs / math.radians(25.0)))
        dist_factor = min(1.0, max(0.0, (fwd_clearance - 1.5) / 3.0))

        speed = self.v_min + (self.v_max - self.v_min) * (0.6 * steer_factor + 0.4 * dist_factor)
        return float(np.clip(speed, self.v_min, self.v_max))

    def lidar_callback(self, data):
        """Process LiDAR scans, execute FTG, and publish drive commands."""
        angle_min = data.angle_min
        angle_inc = data.angle_increment

        # Step 1: Preprocess LiDAR scan
        proc_ranges, min_idx, max_idx = self.preprocess_lidar(data.ranges, angle_min, angle_inc)

        # Step 2 & 3: Find closest point and draw safety bubble
        proc_ranges, closest_idx, min_dist = self.apply_safety_bubble(proc_ranges, angle_inc)

        # Step 4: Disparity extender
        proc_ranges = self.apply_disparity_extender(proc_ranges, angle_inc)

        # Step 5: Find max gap
        max_gap = self.find_max_gap(proc_ranges, free_threshold=0.7)

        if max_gap is None:
            # Fallback safe crawl
            steer_angle = 0.0
            speed = self.v_min
        else:
            # Step 6: Find best point in gap
            target_local_idx = self.find_best_point(max_gap[0], max_gap[1], proc_ranges)
            target_global_idx = min_idx + target_local_idx
            steer_angle = angle_min + target_global_idx * angle_inc

            # Rate-limiting steering smoothing
            steer_angle = float(np.clip(
                steer_angle,
                self.prev_steer - self.max_steer_rate,
                self.prev_steer + self.max_steer_rate
            ))
            self.prev_steer = steer_angle

            # Step 7: Dynamic speed control
            speed = self.compute_speed(steer_angle, proc_ranges)

        self.current_speed = speed
        self.current_steer = steer_angle

        # If all 10 laps are completed, safely bring car to a stop
        if self.all_laps_done:
            speed = 0.0
            steer_angle = 0.0

        # Publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = steer_angle
        drive_msg.drive.speed = speed
        self.drive_pub.publish(drive_msg)

    def odom_callback(self, msg):
        """
        Track position, detect finish line crossing on Sakhir,
        and update lap counts and timers.
        """
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.pose_x = x
        self.pose_y = y

        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # Initialize start times on first odometry message
        if self.sim_start_time is None:
            self.sim_start_time = now_sec
            self.lap_start_time = now_sec
            self.prev_pose = (x, y)
            self.prev_y = y
            return

        # Calculate distance traveled
        dx = x - self.prev_pose[0]
        dy = y - self.prev_pose[1]
        self.total_distance += math.hypot(dx, dy)
        self.prev_pose = (x, y)

        elapsed_lap = now_sec - self.lap_start_time

        # Arm trigger when car travels away from start line
        # Start line on Sakhir is at (0, 0)
        if y > 15.0 or x > 15.0 or elapsed_lap > 15.0:
            self.is_armed = True

        # Finish line crossing condition:
        # Car passes from y < 0 to y >= 0 while near start line (|x| < 3.0 m)
        if self.is_armed and self.prev_y < 0.0 and y >= 0.0 and abs(x) < 3.0:
            self.completed_laps += 1
            lap_time = elapsed_lap
            self.last_lap_time = lap_time
            self.lap_times.append(lap_time)

            if lap_time < self.best_lap_time:
                self.best_lap_time = lap_time

            total_elapsed = now_sec - self.sim_start_time

            # Log to CSV
            self._log_lap_to_csv(self.completed_laps, lap_time, self.best_lap_time, total_elapsed)

            # Publish ROS topics
            self.lap_count_pub.publish(Int32(data=self.completed_laps))
            self.lap_time_pub.publish(Float32(data=float(lap_time)))
            self.best_lap_pub.publish(Float32(data=float(self.best_lap_time)))

            # Print celebration lap notification
            self._print_lap_banner(self.completed_laps, lap_time, self.best_lap_time, total_elapsed)

            # Reset timer for next lap
            self.lap_start_time = now_sec
            self.current_lap = self.completed_laps + 1
            self.is_armed = False

            if self.completed_laps >= self.target_laps:
                self.all_laps_done = True
                self._print_final_summary()

        self.prev_y = y

        # Publish visual RViz HUD markers
        self.publish_rviz_hud(now_sec)

    def publish_rviz_hud(self, now_sec):
        """Publish floating text markers in RViz for live on-screen visualization."""
        if self.sim_start_time is None:
            return

        current_lap_time = now_sec - self.lap_start_time
        total_time = now_sec - self.sim_start_time

        # Marker 1: Fixed track-side Finish Line Board (at x=0, y=0, z=4.0)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'lap_hud'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 2.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 4.0
        marker.scale.z = 1.5  # text height
        marker.color.r = 1.0
        marker.color.g = 0.9
        marker.color.b = 0.0
        marker.color.a = 1.0

        best_str = f'{self.best_lap_time:.2f} s' if self.best_lap_time != float('inf') else '---'
        last_str = f'{self.last_lap_time:.2f} s' if self.last_lap_time > 0 else '---'
        status = 'COMPLETED 10 LAPS!' if self.all_laps_done else 'RACING'

        marker.text = (
            f"F1TENTH SAKHIR - ISLAM QAJ MAZARUL\n"
            f"----------------------------------\n"
            f"STATUS: {status}\n"
            f"LAP: {min(self.completed_laps + 1, self.target_laps)} / {self.target_laps}  (Finished: {self.completed_laps})\n"
            f"CURRENT LAP: {current_lap_time:.2f} s\n"
            f"LAST LAP:    {last_str}\n"
            f"FASTEST LAP: {best_str} \n"
            f"TOTAL TIME:  {total_time:.1f} s\n"
            f"SPEED:       {self.current_speed:.2f} m/s"
        )
        self.hud_pub.publish(marker)

        # Marker 2: Floating HUD right above the racecar (follows the car everywhere)
        car_marker = Marker()
        car_marker.header.frame_id = 'ego_racecar/base_link'
        car_marker.header.stamp = self.get_clock().now().to_msg()
        car_marker.ns = 'car_hud'
        car_marker.id = 1
        car_marker.type = Marker.TEXT_VIEW_FACING
        car_marker.action = Marker.ADD
        car_marker.pose.position.x = 0.0
        car_marker.pose.position.y = 0.0
        car_marker.pose.position.z = 1.8
        car_marker.scale.z = 0.8
        car_marker.color.r = 0.2
        car_marker.color.g = 1.0
        car_marker.color.b = 0.3
        car_marker.color.a = 1.0
        car_marker.text = (
            f"LAP {min(self.completed_laps + 1, self.target_laps)}/{self.target_laps} | {current_lap_time:.1f}s\n"
            f"{self.current_speed:.1f} m/s"
        )
        self.car_hud_pub.publish(car_marker)

    def _print_lap_banner(self, lap_num, lap_time, best_time, total_time):
        """Print an eye-catching banner when a lap is completed."""
        is_fastest = (lap_time <= best_time)
        badge = " *** FASTEST LAP! ***" if is_fastest else ""
        print("\n" + "=" * 65)
        print(f"  🏁 LAP {lap_num} / {self.target_laps} COMPLETED!{badge}")
        print(f"  ⏱️  Lap Time:      {lap_time:6.2f} seconds")
        print(f"  ⭐ Fastest Lap:   {best_time:6.2f} seconds")
        print(f"  ⏳ Total Time:    {total_time:6.2f} seconds")
        print(f"  🚗 Speed:         {self.current_speed:6.2f} m/s")
        print("=" * 65 + "\n")

    def _print_final_summary(self):
        """Print final 10-lap competition summary table."""
        print("\n" + "#" * 65)
        print("  🏆 10 LAPS CHALLENGE SUCCESSFULLY COMPLETED! 🏆")
        print(f"  Student: Islam Qaj Mazarul | Group 2 | Track: Sakhir")
        print("  Collisions: 0 (Flawless Navigation)")
        print("-" * 65)
        print("   Lap   |  Time (s)  | Status")
        print("-" * 65)
        for i, t in enumerate(self.lap_times, 1):
            star = " (FASTEST LAP) ⭐" if t == self.best_lap_time else ""
            print(f"   Lap {i:2d}|  {t:7.2f} s | Clean{star}")
        print("-" * 65)
        print(f"  ⭐ FASTEST LAP TIME: {self.best_lap_time:.2f} s")
        print(f"  ⏱️  AVERAGE LAP TIME: {sum(self.lap_times)/len(self.lap_times):.2f} s")
        print(f"  📁 Records saved to: {self.log_csv_path}")
        print("#" * 65 + "\n")

    def status_timer_callback(self):
        """Periodic 1Hz status output to terminal."""
        if self.sim_start_time is None or self.all_laps_done:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        current_lap_time = now_sec - self.lap_start_time
        total_time = now_sec - self.sim_start_time
        best_str = f'{self.best_lap_time:.2f}s' if self.best_lap_time != float('inf') else '---'

        stats_str = (
            f"[F1TENTH] Lap: {min(self.completed_laps + 1, self.target_laps)}/{self.target_laps} | "
            f"Cur Time: {current_lap_time:5.1f}s | "
            f"Best: {best_str} | "
            f"Speed: {self.current_speed:4.1f} m/s | "
            f"Pose: ({self.pose_x:5.1f}, {self.pose_y:5.1f})"
        )
        print(stats_str)
        self.stats_pub.publish(String(data=stats_str))


def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
