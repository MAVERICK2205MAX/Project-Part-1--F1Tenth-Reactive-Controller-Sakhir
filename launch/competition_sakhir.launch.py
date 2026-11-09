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
            'bubble_hazard_cone_deg': 58.0,
            'bubble_max_dist': 1.8,
            'max_range': 6.0,
            'fov_deg': 72.0,
            'v_min': 2.2,
            'v_max': 8.0,
            'disparity_threshold': 0.7,
            'ema_alpha': 0.28,
            'max_steer_rate': 0.15,
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
