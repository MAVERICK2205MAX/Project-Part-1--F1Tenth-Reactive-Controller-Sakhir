import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to gym_bridge launch file
    gym_bridge_pkg = get_package_share_directory('f1tenth_gym_ros')
    gym_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gym_bridge_pkg, 'launch', 'gym_bridge_launch.py')
        )
    )

    # Reactive Follow-The-Gap controller node (Anti-Wobble Optimized)
    gap_controller_node = Node(
        package='controllers',
        executable='gap_node',
        name='reactive_gap_controller',
        output='screen',
        parameters=[{
            'bubble_radius': 0.35,
            'safety_width': 0.45,
            'bubble_max_dist': 2.0,
            'max_range': 8.0,
            'fov_deg': 70.0,
            'v_min': 2.0,
            'v_max': 7.6,
            'disparity_threshold': 1.2,
            'ema_alpha': 0.20,
            'max_steer_velocity': 2.8,
            'v_steer_base': 3.0,
            'v_steer_pow': 1.0,
            'd_floor': 2.0,
            'd_range': 4.8,
            'target_laps': 10,
            'log_csv_path': 'sakhir_lap_records.csv'
        }]
    )

    # Live Telemetry GUI Overlay (OpenCV dashboard window)
    telemetry_gui_node = Node(
        package='controllers',
        executable='telemetry_gui',
        name='telemetry_gui_overlay',
        output='screen'
    )

    return LaunchDescription([
        gym_bridge_launch,
        gap_controller_node,
        telemetry_gui_node
    ])
