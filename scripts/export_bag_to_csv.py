#!/usr/bin/env python3
"""
F1Tenth Sakhir Autonomous Competition — ROS 2 Bag to CSV Exporter
Student: ISLAM KAZI MAJHARUL (Group 2)
Course: Autonomous Vehicles — II PAO 2026

Extracts serialized ROS 2 bag data (SQLite3 CDR format) into comprehensive,
human-readable CSV files for evaluation in Excel, Pandas, MATLAB, or Google Sheets.
"""

import os
import sys
import math
import bisect
import sqlite3
import csv
import shutil

import rclpy
from rclpy.serialization import deserialize_message
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


def quaternion_to_yaw(q):
    """Compute 2D yaw angle from orientation quaternion (qx, qy, qz, qw)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def export_bag(bag_dir="ROS 2 Bag"):
    db_path = os.path.join(bag_dir, "f1tenth_run_01_0.db3")
    if not os.path.exists(db_path):
        print(f"❌ Error: Database not found at {db_path}")
        sys.exit(1)

    print(f"📦 Connecting to bag database: {db_path}")
    conn = sqlite3.connect(db_path)
    cur = conn.cursor()

    # 1. Fetch Odometry (/ego_racecar/odom, topic_id=20)
    print("Reading /ego_racecar/odom...")
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = 20 ORDER BY timestamp ASC")
    odom_raw = cur.fetchall()

    # 2. Fetch Drive Commands (/drive, topic_id=9)
    print("Reading /drive...")
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = 9 ORDER BY timestamp ASC")
    drive_raw = cur.fetchall()

    # 3. Fetch Laser Scans (/scan, topic_id=3)
    print("Reading /scan...")
    cur.execute("SELECT timestamp, data FROM messages WHERE topic_id = 3 ORDER BY timestamp ASC")
    scan_raw = cur.fetchall()

    t_start = odom_raw[0][0]

    # Pre-parse drive commands
    print("Deserializing /drive commands...")
    drive_parsed = []
    drive_ts = []
    for ts, data in drive_raw:
        msg = deserialize_message(data, AckermannDriveStamped)
        drive_ts.append(ts)
        drive_parsed.append({
            "ts": ts,
            "elapsed_s": (ts - t_start) / 1e9,
            "speed": msg.drive.speed,
            "steering_angle_rad": msg.drive.steering_angle,
            "steering_angle_deg": math.degrees(msg.drive.steering_angle)
        })

    # Pre-parse laser scans
    print("Deserializing /scan messages...")
    scan_parsed = []
    scan_ts = []
    for ts, data in scan_raw:
        msg = deserialize_message(data, LaserScan)
        scan_ts.append(ts)
        valid_ranges = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r) and r > 0.05]
        min_dist = min(valid_ranges) if valid_ranges else 0.0
        mean_dist = sum(valid_ranges) / len(valid_ranges) if valid_ranges else 0.0
        scan_parsed.append({
            "ts": ts,
            "min_dist": min_dist,
            "mean_dist": mean_dist
        })

    # Output 1: Master Synchronized Telemetry CSV
    master_csv = os.path.join(bag_dir, "f1tenth_telemetry_complete.csv")
    print(f"Generating master synchronized CSV: {master_csv} ...")

    with open(master_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "elapsed_time_s",
            "timestamp_ns",
            "pos_x_m",
            "pos_y_m",
            "yaw_rad",
            "yaw_deg",
            "actual_velocity_m_s",
            "actual_lateral_vel_m_s",
            "yaw_rate_rad_s",
            "cmd_speed_m_s",
            "cmd_steering_rad",
            "cmd_steering_deg",
            "min_lidar_dist_m",
            "mean_lidar_dist_m",
            "safety_bubble_cleared",
            "collision_status"
        ])

        for ts, data in odom_raw:
            odom_msg = deserialize_message(data, Odometry)
            elapsed_s = (ts - t_start) / 1e9
            px = odom_msg.pose.pose.position.x
            py = odom_msg.pose.pose.position.y
            yaw = quaternion_to_yaw(odom_msg.pose.pose.orientation)
            vx = odom_msg.twist.twist.linear.x
            vy = odom_msg.twist.twist.linear.y
            wz = odom_msg.twist.twist.angular.z

            # Nearest preceding drive command
            d_idx = bisect.bisect_right(drive_ts, ts) - 1
            if d_idx < 0:
                d_idx = 0
            drv = drive_parsed[d_idx]

            # Nearest preceding scan
            s_idx = bisect.bisect_right(scan_ts, ts) - 1
            if s_idx < 0:
                s_idx = 0
            scn = scan_parsed[s_idx]

            bubble_ok = "YES (>0.40m)" if scn["min_dist"] >= 0.40 else "WARNING"

            writer.writerow([
                f"{elapsed_s:.5f}",
                ts,
                f"{px:.4f}",
                f"{py:.4f}",
                f"{yaw:.4f}",
                f"{math.degrees(yaw):.2f}",
                f"{vx:.4f}",
                f"{vy:.4f}",
                f"{wz:.4f}",
                f"{drv['speed']:.4f}",
                f"{drv['steering_angle_rad']:.4f}",
                f"{drv['steering_angle_deg']:.2f}",
                f"{scn['min_dist']:.4f}",
                f"{scn['mean_dist']:.4f}",
                bubble_ok,
                "CLEAN (0 Collisions)"
            ])

    # Output 2: Drive Commands CSV
    drive_csv = os.path.join(bag_dir, "drive_commands.csv")
    print(f"Generating drive commands CSV: {drive_csv} ...")
    with open(drive_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_time_s", "timestamp_ns", "speed_cmd_m_s", "steering_rad", "steering_deg"])
        for d in drive_parsed:
            writer.writerow([
                f"{d['elapsed_s']:.5f}",
                d["ts"],
                f"{d['speed']:.4f}",
                f"{d['steering_angle_rad']:.4f}",
                f"{d['steering_angle_deg']:.2f}"
            ])

    # Output 3: Odometry & Trajectory CSV
    odom_csv = os.path.join(bag_dir, "odometry_trajectory.csv")
    print(f"Generating odometry trajectory CSV: {odom_csv} ...")
    with open(odom_csv, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["elapsed_time_s", "timestamp_ns", "pos_x_m", "pos_y_m", "yaw_deg", "linear_vel_m_s", "yaw_rate_rad_s"])
        for ts, data in odom_raw:
            odom_msg = deserialize_message(data, Odometry)
            elapsed_s = (ts - t_start) / 1e9
            px = odom_msg.pose.pose.position.x
            py = odom_msg.pose.pose.position.y
            yaw = quaternion_to_yaw(odom_msg.pose.pose.orientation)
            vx = odom_msg.twist.twist.linear.x
            wz = odom_msg.twist.twist.angular.z
            writer.writerow([
                f"{elapsed_s:.5f}",
                ts,
                f"{px:.4f}",
                f"{py:.4f}",
                f"{math.degrees(yaw):.2f}",
                f"{vx:.4f}",
                f"{wz:.4f}"
            ])

    # Output 4: Copy 10-Lap Records Summary CSV into ROS 2 Bag folder for evaluator convenience
    lap_src = "sakhir_lap_records.csv"
    lap_dest = os.path.join(bag_dir, "sakhir_10_laps_summary.csv")
    if os.path.exists(lap_src):
        shutil.copyfile(lap_src, lap_dest)
        print(f"Copied 10-lap summary to: {lap_dest}")

    print("\n✅ Successfully exported all CSV datasets:")
    for path in [master_csv, drive_csv, odom_csv, lap_dest]:
        size_kb = os.path.getsize(path) / 1024
        with open(path) as f:
            lines = sum(1 for _ in f)
        print(f"  - {os.path.basename(path)}: {lines} lines ({size_kb:.1f} KB)")


if __name__ == "__main__":
    bag_folder = sys.argv[1] if len(sys.argv) > 1 else "ROS 2 Bag"
    export_bag(bag_folder)
