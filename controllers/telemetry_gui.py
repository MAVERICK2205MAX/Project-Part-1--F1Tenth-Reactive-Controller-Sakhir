#!/usr/bin/env python3
"""
F1Tenth Competition Live Telemetry HUD Overlay
Track: Sakhir | Student: Islam Qaj Mazarul (Group 2)

Renders a live telemetry HUD window displaying:
- Current Lap Counter (e.g. LAP 3 / 10)
- Live Lap Stopwatch
- Best (Fastest) Lap Time
- Last Lap Time & Total Time
- Real-time Speed & Steering Angle gauges
- Per-lap split history table
- Clean collision status badge
"""

import cv2
import numpy as np
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Float32, String


class TelemetryGUI(Node):
    def __init__(self):
        super().__init__('telemetry_gui')

        # Subscriptions
        self.create_subscription(Int32, '/lap_count', self.lap_count_cb, 10)
        self.create_subscription(Float32, '/lap_time', self.lap_time_cb, 10)
        self.create_subscription(Float32, '/best_lap_time', self.best_lap_cb, 10)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_cb, 10)
        self.create_subscription(AckermannDriveStamped, '/drive', self.drive_cb, 10)

        # State
        self.completed_laps = 0
        self.target_laps = 10
        self.current_lap_start_time = None
        self.sim_start_time = None
        self.best_lap = float('inf')
        self.last_lap = 0.0
        self.lap_splits = []
        self.speed = 0.0
        self.steering = 0.0
        self.pose = (0.0, 0.0)

        # GUI timer: 20 Hz refresh (50ms)
        self.timer = self.create_timer(0.05, self.render_frame)

    def lap_count_cb(self, msg):
        self.completed_laps = msg.data

    def lap_time_cb(self, msg):
        self.last_lap = msg.data
        self.lap_splits.append((self.completed_laps, msg.data))
        self.current_lap_start_time = time.time()

    def best_lap_cb(self, msg):
        self.best_lap = msg.data

    def odom_cb(self, msg):
        now = time.time()
        if self.sim_start_time is None:
            self.sim_start_time = now
            self.current_lap_start_time = now
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = np.hypot(vx, vy)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def drive_cb(self, msg):
        self.steering = msg.drive.steering_angle

    def render_frame(self):
        width, height = 620, 520
        canvas = np.zeros((height, width, 3), dtype=np.uint8)

        # Background gradient: dark carbon / race blue
        canvas[:] = (22, 22, 26)

        # Header bar
        cv2.rectangle(canvas, (0, 0), (width, 55), (38, 38, 48), -1)
        cv2.rectangle(canvas, (0, 53), (width, 55), (0, 200, 255), -1)

        cv2.putText(canvas, "F1TENTH SAKHIR TELEMETRY", (15, 28),
                    cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, "DRIVER: Islam Qaj Mazarul  |  GROUP 2", (15, 47),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (180, 220, 255), 1)

        # Current lap calculation
        cur_lap_num = min(self.completed_laps + 1, self.target_laps)
        if self.current_lap_start_time is not None:
            cur_lap_time = time.time() - self.current_lap_start_time
        else:
            cur_lap_time = 0.0

        if self.sim_start_time is not None:
            total_time = time.time() - self.sim_start_time
        else:
            total_time = 0.0

        # Status badge (top right)
        if self.completed_laps >= self.target_laps:
            status_text = "FINISH (10 LAPS)"
            status_col = (0, 255, 120)
        else:
            status_text = "RACING"
            status_col = (0, 215, 255)
        cv2.rectangle(canvas, (width - 170, 12), (width - 15, 42), status_col, 2)
        cv2.putText(canvas, status_text, (width - 160, 33),
                    cv2.FONT_HERSHEY_DUPLEX, 0.52, status_col, 1)

        # Primary Stats Cards
        # Box 1: LAP COUNTER
        cv2.rectangle(canvas, (15, 70), (195, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, 70), (195, 155), (70, 70, 85), 1)
        cv2.putText(canvas, "LAP PROGRESS", (25, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 170), 1)
        cv2.putText(canvas, f"{cur_lap_num} / {self.target_laps}", (25, 130),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1, (0, 255, 160), 2)
        cv2.putText(canvas, f"Completed: {self.completed_laps}", (25, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 150), 1)

        # Box 2: CURRENT LAP TIMER
        cv2.rectangle(canvas, (210, 70), (410, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (210, 70), (410, 155), (70, 70, 85), 1)
        cv2.putText(canvas, "CURRENT LAP TIME", (220, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 170), 1)
        mins = int(cur_lap_time // 60)
        secs = cur_lap_time % 60
        timer_str = f"{mins:02d}:{secs:05.2f}"
        cv2.putText(canvas, timer_str, (220, 130), cv2.FONT_HERSHEY_DUPLEX, 1.05, (255, 255, 255), 2)
        cv2.putText(canvas, f"Total: {total_time:5.1f}s", (220, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 150), 1)

        # Box 3: FASTEST LAP (BEST)
        cv2.rectangle(canvas, (425, 70), (width - 15, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (425, 70), (width - 15, 155), (0, 180, 255), 1)
        cv2.putText(canvas, "FASTEST LAP", (435, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)
        if self.best_lap != float('inf'):
            b_mins = int(self.best_lap // 60)
            b_secs = self.best_lap % 60
            best_str = f"{b_mins:02d}:{b_secs:05.2f}"
            cv2.putText(canvas, best_str, (435, 130), cv2.FONT_HERSHEY_DUPLEX, 0.95, (0, 215, 255), 2)
            cv2.putText(canvas, f"Record: {self.best_lap:.2f}s", (435, 148),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
        else:
            cv2.putText(canvas, "--:--.--", (435, 130), cv2.FONT_HERSHEY_DUPLEX, 0.95, (100, 100, 110), 2)
            cv2.putText(canvas, "Establishing...", (435, 148),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 110), 1)

        # Speed and Steering Gauge section
        cv2.rectangle(canvas, (15, 170), (width - 15, 245), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, 170), (width - 15, 245), (70, 70, 85), 1)

        # Speed Bar
        cv2.putText(canvas, f"SPEED: {self.speed:4.2f} m/s", (25, 195),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)
        max_v = 8.0
        bar_w = 240
        fill_w = int(min(1.0, max(0.0, self.speed / max_v)) * bar_w)
        cv2.rectangle(canvas, (25, 205), (25 + bar_w, 225), (50, 50, 60), -1)
        col = (0, 255, 160) if self.speed < 6.0 else (0, 200, 255)
        cv2.rectangle(canvas, (25, 205), (25 + fill_w, 225), col, -1)
        cv2.rectangle(canvas, (25, 205), (25 + bar_w, 225), (100, 100, 110), 1)

        # Steering Angle
        steer_deg = math.degrees(self.steering)
        cv2.putText(canvas, f"STEER: {steer_deg:+5.1f} deg", (330, 195),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 220, 220), 1)
        # Steer bar from center
        cv2.rectangle(canvas, (330, 205), (330 + 240, 225), (50, 50, 60), -1)
        center_x = 330 + 120
        steer_fill = int((steer_deg / 30.0) * 120)
        steer_fill = max(-120, min(120, steer_fill))
        cv2.line(canvas, (center_x, 203), (center_x, 227), (255, 255, 255), 2)
        if steer_fill > 0:
            cv2.rectangle(canvas, (center_x, 205), (center_x + steer_fill, 225), (255, 160, 0), -1)
        else:
            cv2.rectangle(canvas, (center_x + steer_fill, 205), (center_x, 225), (255, 160, 0), -1)
        cv2.rectangle(canvas, (330, 205), (330 + 240, 225), (100, 100, 110), 1)

        # Lap-by-Lap Split History Table
        cv2.rectangle(canvas, (15, 260), (width - 15, 475), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, 260), (width - 15, 475), (70, 70, 85), 1)
        cv2.putText(canvas, "LAP SPLIT HISTORY", (25, 282),
                    cv2.FONT_HERSHEY_DUPLEX, 0.5, (220, 220, 220), 1)

        # Table header
        cv2.line(canvas, (25, 292), (width - 25, 292), (70, 70, 85), 1)
        cv2.putText(canvas, "Lap", (35, 308), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Split Time", (120, 308), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Diff vs Best", (260, 308), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Status", (430, 308), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.line(canvas, (25, 315), (width - 25, 315), (55, 55, 70), 1)

        # Render rows (show up to 6 most recent)
        recent_splits = self.lap_splits[-6:] if self.lap_splits else []
        y_pos = 335
        for l_num, l_time in recent_splits:
            is_best = (l_time == self.best_lap)
            row_col = (0, 215, 255) if is_best else (220, 220, 220)
            diff = l_time - self.best_lap if self.best_lap != float('inf') else 0.0
            diff_str = f"+{diff:4.2f}s" if diff > 0.001 else "BEST LAP"

            cv2.putText(canvas, f"Lap {l_num:02d}", (35, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, row_col, 1)
            cv2.putText(canvas, f"{l_time:6.2f} s", (120, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, row_col, 1)
            cv2.putText(canvas, diff_str, (260, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 215, 255) if is_best else (160, 160, 170), 1)
            badge_str = "CLEAN (0 COL)" if not is_best else "CLEAN - RECORD"
            cv2.putText(canvas, badge_str, (430, y_pos),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)
            y_pos += 24

        if not self.lap_splits:
            cv2.putText(canvas, "Awaiting completion of Lap 1...", (160, 380),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 120, 130), 1)

        # Footer
        cv2.rectangle(canvas, (0, height - 32), (width, height), (38, 38, 48), -1)
        cv2.putText(canvas, "Track: Sakhir  |  Algorithm: Follow-The-Gap + Disparity Extender", (15, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 170), 1)
        cv2.putText(canvas, "Autonomous Vehicles Project", (width - 200, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)

        cv2.imshow("F1Tenth Live Telemetry - Sakhir", canvas)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    telemetry_gui = TelemetryGUI()
    try:
        rclpy.spin(telemetry_gui)
    except KeyboardInterrupt:
        pass
    finally:
        telemetry_gui.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
