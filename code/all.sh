#!/bin/bash

SCRIPT_DIR=~/catkin_ws/src/Project_drone/code

killall gnome-terminal

gnome-terminal -- bash -c "roscore; exec bash"

gnome-terminal -- bash -c "sleep 3; $SCRIPT_DIR/1q.sh; exec bash"
gnome-terminal -- bash -c "sleep 10; $SCRIPT_DIR/2w.sh; exec bash"
gnome-terminal -- bash -c "sleep 8; $SCRIPT_DIR/3e.sh; exec bash"
gnome-terminal -- bash -c "sleep 5; $SCRIPT_DIR/5t_gone3.sh; exec bash"
gnome-terminal -- bash -c "sleep 15; $SCRIPT_DIR/6r.sh; exec bash"


