#!/usr/bin/env bash
# ==============================================================================
# F1Tenth Sakhir Autonomous Competition — ROS 2 Bag Player
# Student: ISLAM KAZI MAJHARUL (Group 2)
# Usage: ./scripts/play_bag.sh [bag_folder_name] [playback_rate] [extra_flags]
# Example: ./scripts/play_bag.sh "ROS 2 Bag" 0.5
#          ./scripts/play_bag.sh "ROS 2 Bag" 1.0 --loop
# ==============================================================================

BAG_NAME="${1:-ROS 2 Bag}"
RATE="${2:-1.0}"
shift 2 2>/dev/null || shift 1 2>/dev/null || true
EXTRA_ARGS="$@"

source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/foxy/setup.bash 2>/dev/null
source /home/admin1/sim_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

if [ ! -d "${BAG_NAME}" ]; then
    echo "❌ Error: Bag directory '${BAG_NAME}' not found."
    exit 1
fi

echo "======================================================================"
echo "  ▶️  Replaying ROS 2 Bag: ${BAG_NAME}"
echo "  Playback Rate: ${RATE}x"
if [ -n "${EXTRA_ARGS}" ]; then
    echo "  Extra Options: ${EXTRA_ARGS}"
fi
echo "  Controls: [Space] Pause/Resume | [Right Arrow] Step | [Up/Down] Rate"
echo "======================================================================"

ros2 bag info "${BAG_NAME}"
echo "----------------------------------------------------------------------"
ros2 bag play "${BAG_NAME}" --rate "${RATE}" ${EXTRA_ARGS}
