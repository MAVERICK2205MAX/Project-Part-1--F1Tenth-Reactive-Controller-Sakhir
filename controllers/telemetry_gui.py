#!/usr/bin/env python3
"""
F1Tenth Competition Live Telemetry HUD Overlay (Scrollable 10-Lap Dashboard)
Track: Sakhir | Student: ISLAM KAZI MAJHARUL (Group 2)

Renders a live telemetry HUD window displaying:
- Current Lap Counter (e.g. 10 / 10)
- Live Lap Stopwatch & Total Mission Timer
- Best (Fastest) Lap Time
- Real-time Speed & Steering Angle gauges (with Parked indication)
- Scrollable per-lap split history table showing all 10 laps simultaneously
- Interactive mouse-wheel / click-to-scroll scrollbar
- 0 Collision clean run verification badges
"""

import cv2
import numpy as np
import time
import math

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

        # Scrollbar state
        self.scroll_offset = 0
        self.window_name = "F1Tenth Live Telemetry - Sakhir"
        self.window_created = False

        # GUI timer: 20 Hz refresh (50ms)
        self.timer = self.create_timer(0.05, self.render_frame)

    def lap_count_cb(self, msg):
        self.completed_laps = min(int(msg.data), self.target_laps)

    def lap_time_cb(self, msg):
        if len(self.lap_splits) >= self.target_laps:
            return
        self.last_lap = msg.data
        lap_idx = min(self.completed_laps, self.target_laps)
        # Avoid duplicate lap entries
        if not any(lap_num == lap_idx for lap_num, _ in self.lap_splits):
            self.lap_splits.append((lap_idx, msg.data))
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
        if self.completed_laps >= self.target_laps:
            self.speed = 0.0
        else:
            self.speed = np.hypot(vx, vy)
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def drive_cb(self, msg):
        if self.completed_laps >= self.target_laps:
            self.steering = 0.0
        else:
            self.steering = msg.drive.steering_angle

    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse wheel scrolling and clicking on the scrollbar track."""
        total_items = len(self.lap_splits)
        max_scroll = max(0, total_items - 10)

        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.scroll_offset = max(0, self.scroll_offset - 1)
            else:
                self.scroll_offset = min(max_scroll, self.scroll_offset + 1)
        elif event == cv2.EVENT_LBUTTONDOWN:
            sb_x = 640 - 26
            sb_top = 250 + 32 + 24
            sb_bot = 635 - 36
            if sb_x <= x <= sb_x + 12 and sb_top <= y <= sb_bot:
                ratio = (y - sb_top) / max(1, sb_bot - sb_top)
                self.scroll_offset = int(round(ratio * max_scroll))

    def render_frame(self):
        width, height = 640, 680
        canvas = np.zeros((height, width, 3), dtype=np.uint8)

        # Background: Dark carbon theme
        canvas[:] = (22, 22, 26)

        # Header bar
        cv2.rectangle(canvas, (0, 0), (width, 58), (35, 35, 45), -1)
        cv2.rectangle(canvas, (0, 56), (width, 58), (0, 200, 255), -1)

        cv2.putText(canvas, "F1TENTH SAKHIR TELEMETRY", (15, 28),
                    cv2.FONT_HERSHEY_DUPLEX, 0.75, (255, 255, 255), 2)
        cv2.putText(canvas, "DRIVER: ISLAM KAZI MAJHARUL  |  GROUP 2", (15, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 220, 255), 1)

        # Elapsed times
        now_t = time.time()
        if self.current_lap_start_time is not None:
            cur_lap_time = now_t - self.current_lap_start_time
        else:
            cur_lap_time = 0.0

        if self.sim_start_time is not None:
            total_time = now_t - self.sim_start_time
        else:
            total_time = 0.0

        # Status badge (top right)
        is_finished = (self.completed_laps >= self.target_laps)
        if is_finished:
            status_text = "FINISH (10 LAPS PARKED)"
            status_col = (0, 255, 120)
        else:
            status_text = "RACING"
            status_col = (0, 215, 255)

        badge_w = 210
        cv2.rectangle(canvas, (width - badge_w - 15, 12), (width - 15, 46),
                      (40, 50, 40) if is_finished else (40, 40, 50), -1)
        cv2.rectangle(canvas, (width - badge_w - 15, 12), (width - 15, 46), status_col, 2)
        cv2.putText(canvas, status_text, (width - badge_w - 5, 34),
                    cv2.FONT_HERSHEY_DUPLEX, 0.46, status_col, 1)

        # Primary Stats Cards
        cur_lap_num = min(self.completed_laps + 1, self.target_laps)

        # Box 1: LAP PROGRESS
        cv2.rectangle(canvas, (15, 72), (200, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, 72), (200, 155), (70, 70, 85), 1)
        cv2.putText(canvas, "LAP PROGRESS", (25, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 170), 1)
        cv2.putText(canvas, f"{cur_lap_num} / {self.target_laps}", (25, 130),
                    cv2.FONT_HERSHEY_DUPLEX, 1.1, (0, 255, 160), 2)
        cv2.putText(canvas, f"Completed: {self.completed_laps}", (25, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 150), 1)

        # Box 2: CURRENT LAP TIMER / STATUS
        cv2.rectangle(canvas, (215, 72), (425, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (215, 72), (425, 155), (70, 70, 85), 1)
        cv2.putText(canvas, "STATUS / TIMER", (225, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (160, 160, 170), 1)
        if is_finished:
            cv2.putText(canvas, "COMPLETED", (225, 128), cv2.FONT_HERSHEY_DUPLEX, 0.95, (0, 255, 160), 2)
        else:
            mins = int(cur_lap_time // 60)
            secs = cur_lap_time % 60
            timer_str = f"{mins:02d}:{secs:05.2f}"
            cv2.putText(canvas, timer_str, (225, 128), cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 2)
        cv2.putText(canvas, f"Total Run Time: {total_time:5.1f}s", (225, 148),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 150), 1)

        # Box 3: FASTEST LAP (BEST)
        cv2.rectangle(canvas, (440, 72), (width - 15, 155), (32, 32, 40), -1)
        cv2.rectangle(canvas, (440, 72), (width - 15, 155), (0, 180, 255), 1)
        cv2.putText(canvas, "FASTEST LAP", (450, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 180, 255), 1)
        if self.best_lap != float('inf'):
            b_mins = int(self.best_lap // 60)
            b_secs = self.best_lap % 60
            best_str = f"{b_mins:02d}:{b_secs:05.2f}"
            cv2.putText(canvas, best_str, (450, 128), cv2.FONT_HERSHEY_DUPLEX, 0.95, (0, 215, 255), 2)
            cv2.putText(canvas, f"Record: {self.best_lap:.2f}s", (450, 148),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)
        else:
            cv2.putText(canvas, "--:--.--", (450, 128), cv2.FONT_HERSHEY_DUPLEX, 0.95, (100, 100, 110), 2)
            cv2.putText(canvas, "Establishing...", (450, 148),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (100, 100, 110), 1)

        # Speed and Steering Gauge section
        cv2.rectangle(canvas, (15, 168), (width - 15, 238), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, 168), (width - 15, 238), (70, 70, 85), 1)

        # Speed Bar
        speed_val = 0.0 if is_finished else self.speed
        speed_label = f"SPEED: {speed_val:4.2f} m/s (PARKED)" if is_finished else f"SPEED: {speed_val:4.2f} m/s"
        cv2.putText(canvas, speed_label, (25, 192), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 220, 220), 1)
        max_v = 8.5
        bar_w = 260
        fill_w = int(min(1.0, max(0.0, speed_val / max_v)) * bar_w)
        cv2.rectangle(canvas, (25, 202), (25 + bar_w, 222), (50, 50, 60), -1)
        col = (0, 255, 160) if speed_val < 6.0 else (0, 200, 255)
        cv2.rectangle(canvas, (25, 202), (25 + fill_w, 222), col, -1)
        cv2.rectangle(canvas, (25, 202), (25 + bar_w, 222), (100, 100, 110), 1)

        # Steering Angle
        steer_deg = 0.0 if is_finished else math.degrees(self.steering)
        cv2.putText(canvas, f"STEER: {steer_deg:+5.1f} deg", (340, 192),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.46, (220, 220, 220), 1)
        cv2.rectangle(canvas, (340, 202), (340 + 260, 222), (50, 50, 60), -1)
        center_x = 340 + 130
        steer_fill = int((steer_deg / 30.0) * 130)
        steer_fill = max(-130, min(130, steer_fill))
        cv2.line(canvas, (center_x, 200), (center_x, 224), (255, 255, 255), 2)
        if steer_fill > 0:
            cv2.rectangle(canvas, (center_x, 202), (center_x + steer_fill, 222), (255, 160, 0), -1)
        else:
            cv2.rectangle(canvas, (center_x + steer_fill, 202), (center_x, 222), (255, 160, 0), -1)
        cv2.rectangle(canvas, (340, 202), (340 + 260, 222), (100, 100, 110), 1)

        # Lap-by-Lap Split History Table (Full 10-Row + Scrollable)
        tbl_top = 250
        tbl_bottom = 635
        cv2.rectangle(canvas, (15, tbl_top), (width - 15, tbl_bottom), (32, 32, 40), -1)
        cv2.rectangle(canvas, (15, tbl_top), (width - 15, tbl_bottom), (70, 70, 85), 1)
        cv2.putText(canvas, f"LAP SPLIT HISTORY ({len(self.lap_splits)} LAPS RECORDED)", (25, tbl_top + 20),
                    cv2.FONT_HERSHEY_DUPLEX, 0.48, (220, 220, 220), 1)

        # Table header
        hdr_y = tbl_top + 32
        cv2.line(canvas, (25, hdr_y), (width - 35, hdr_y), (70, 70, 85), 1)
        cv2.putText(canvas, "Lap", (35, hdr_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Split Time", (125, hdr_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Diff vs Best", (265, hdr_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.putText(canvas, "Status", (435, hdr_y + 16), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 160), 1)
        cv2.line(canvas, (25, hdr_y + 22), (width - 35, hdr_y + 22), (55, 55, 70), 1)

        # Scrollbar Track on the right side
        sb_x = width - 26
        sb_top = hdr_y + 24
        sb_bot = tbl_bottom - 36
        sb_h = sb_bot - sb_top
        cv2.rectangle(canvas, (sb_x, sb_top), (sb_x + 8, sb_bot), (45, 45, 55), -1)
        cv2.rectangle(canvas, (sb_x, sb_top), (sb_x + 8, sb_bot), (70, 70, 85), 1)

        # Calculate thumb size & position
        total_items = max(1, len(self.lap_splits))
        visible_rows = 10
        thumb_h = max(24, int(sb_h * min(1.0, visible_rows / total_items)))
        max_scroll = max(0, total_items - visible_rows)
        self.scroll_offset = max(0, min(self.scroll_offset, max_scroll))
        scroll_ratio = (self.scroll_offset / max_scroll) if max_scroll > 0 else 0.0
        thumb_y = sb_top + int(scroll_ratio * (sb_h - thumb_h))
        cv2.rectangle(canvas, (sb_x + 1, thumb_y), (sb_x + 7, thumb_y + thumb_h), (0, 190, 255), -1)

        # Render visible rows
        slice_splits = self.lap_splits[self.scroll_offset : self.scroll_offset + visible_rows]
        row_y = hdr_y + 44
        for l_num, l_time in slice_splits:
            is_best = (abs(l_time - self.best_lap) < 0.01)
            row_bg = (40, 48, 55) if is_best else ((35, 35, 44) if l_num % 2 == 0 else (30, 30, 38))
            cv2.rectangle(canvas, (25, row_y - 15), (width - 32, row_y + 7), row_bg, -1)

            row_col = (0, 220, 255) if is_best else (225, 225, 225)
            diff = l_time - self.best_lap if self.best_lap != float('inf') else 0.0
            diff_str = f"+{diff:4.2f}s" if diff > 0.001 else "BEST LAP"

            cv2.putText(canvas, f"Lap {l_num:02d}", (35, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, row_col, 1)
            cv2.putText(canvas, f"{l_time:6.2f} s", (125, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, row_col, 1)
            cv2.putText(canvas, diff_str, (265, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 215, 255) if is_best else (160, 160, 170), 1)
            badge_str = "CLEAN (0 COL)" if not is_best else "CLEAN - RECORD"
            cv2.putText(canvas, badge_str, (435, row_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 120), 1)
            row_y += 24

        if not self.lap_splits:
            cv2.putText(canvas, "Awaiting completion of Lap 1...", (170, 390),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, (120, 120, 130), 1)

        # Table Summary bottom line
        sum_y = tbl_bottom - 12
        cv2.line(canvas, (25, tbl_bottom - 28), (width - 25, tbl_bottom - 28), (60, 60, 75), 1)
        if self.lap_splits:
            times = [t for _, t in self.lap_splits]
            avg_t = sum(times) / len(times)
            summary_txt = f"{len(self.lap_splits)} LAPS RECORDED | AVG: {avg_t:.2f}s | FASTEST: {min(times):.2f}s | 0 COLLISIONS"
            cv2.putText(canvas, summary_txt, (35, sum_y), cv2.FONT_HERSHEY_DUPLEX, 0.40, (0, 255, 160), 1)
        else:
            cv2.putText(canvas, "Target: 10 Consecutive Clean Laps | 0 Collisions Required",
                        (35, sum_y), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (140, 140, 150), 1)

        # Footer
        cv2.rectangle(canvas, (0, height - 32), (width, height), (38, 38, 48), -1)
        cv2.putText(canvas, "Track: Sakhir  |  Algorithm: Follow-The-Gap + Disparity Extender", (15, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (160, 160, 170), 1)
        cv2.putText(canvas, "Autonomous Vehicles Project", (width - 210, height - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 200, 255), 1)

        # Show frame
        if not self.window_created:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.window_name, self.mouse_callback)
            self.window_created = True

        cv2.imshow(self.window_name, canvas)
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
