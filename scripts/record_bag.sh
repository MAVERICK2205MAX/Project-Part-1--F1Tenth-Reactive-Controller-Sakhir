#!/usr/bin/env bash
# ==============================================================================
# F1Tenth Sakhir Autonomous Competition — ROS 2 Bag Recorder
# Student: ISLAM KAZI MAJHARUL (Group 2)
# Usage: ./scripts/record_bag.sh [bag_folder_name]
# Example: ./scripts/record_bag.sh f1tenth_run_01
# ==============================================================================

BAG_NAME="${1:-f1tenth_run_01}"

if [ -d "${BAG_NAME}" ]; then
    echo "⚠️ Warning: Directory '${BAG_NAME}' already exists. ROS 2 will not overwrite existing bags."
    BAG_NAME="${BAG_NAME}_$(date +%H%M%S)"
    echo "➡️ Using new unique folder name: ${BAG_NAME}"
fi

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
source /home/admin1/sim_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "======================================================================"
echo "  📹 Recording ROS 2 Bag for F1Tenth Sakhir Simulation"
echo "  Output Directory: ${BAG_NAME}"
echo "  Topics: All available topics (-a)"
echo "  Press [Ctrl + C] to stop recording cleanly."
echo "======================================================================"

ros2 bag record -a -o "${BAG_NAME}"
