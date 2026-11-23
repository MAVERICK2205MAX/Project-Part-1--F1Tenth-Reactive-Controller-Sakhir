#!/usr/bin/env bash
# ==============================================================================
# F1Tenth Sakhir Autonomous Competition — ROS 2 Bag Recorder
# Student: ISLAM KAZI MAJHARUL (Group 2)
# Usage: ./scripts/record_bag.sh [bag_folder_name] [topics...]
# Example: ./scripts/record_bag.sh f1tenth_run_01
#          ./scripts/record_bag.sh f1tenth_run_01 /scan /ego_racecar/odom /drive
#          ./scripts/record_bag.sh f1tenth_run_01 -a
# ==============================================================================

BAG_NAME="${1:-f1tenth_run_01}"
shift 1 2>/dev/null || true
CUSTOM_TOPICS="$@"

if [ -z "${CUSTOM_TOPICS}" ]; then
    TOPICS="/scan /ego_racecar/odom /drive"
else
    TOPICS="${CUSTOM_TOPICS}"
fi

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
echo "  Target Topics:    ${TOPICS}"
echo "  Press [Ctrl + C] to stop recording cleanly."
echo "======================================================================"

if [ "${TOPICS}" = "-a" ]; then
    ros2 bag record -a -o "${BAG_NAME}"
else
    ros2 bag record -o "${BAG_NAME}" ${TOPICS}
fi
