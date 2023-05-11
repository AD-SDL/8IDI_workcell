#!/bin/bash

session="nodes"
tmux new-session -d -s $session
tmux set -g mouse on

window=0
tmux new-window -t $session:$window -n '8_ID_I_ur3'
tmux send-keys -t $session:$window 'source ~/wei_ws/install/setup.bash' C-m
tmux send-keys -t $session:$window 'ros2 launch 8IDI_client 8IDI_client.launch.py' C-m


tmux attach-session -t $session

