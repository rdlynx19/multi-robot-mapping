#!/bin/bash

SESSION_NAME="start_offboard"

tmux new-session -d -s $SESSION_NAME

tmux split-window -v -t $SESSION_NAME

tmux send-keys -t $SESSION_NAME:0.0 "sudo MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600" C-m

tmux send-keys -t $SESSION_NAME:0.1 "ros2 launch rpi_control offb_px4.launch.xml" C-m

tmux attach-session -t $SESSION_NAME


