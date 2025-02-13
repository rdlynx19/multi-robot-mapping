#!/bin/bash

ZENOH_SCRIPT="/home/itadori/zenoh_router.sh"

tmux split-window -v

tmux send-keys -t 1 "$ZENOH_SCRIPT" C-m

tmux send-keys -t 0 "ros2 launch pi_mapping depthai.launch.py" C-m

