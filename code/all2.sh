#!/bin/bash

SCRIPT_DIR=~/catkin_ws/src/Project_drone/code

killall gnome-terminal

gnome-terminal -- bash -c "roscore; exec bash"

gnome-terminal -- bash -c "sleep 3; $SCRIPT_DIR/1q.sh; exec bash"
gnome-terminal -- bash -c "sleep 10; $SCRIPT_DIR/2w.sh; exec bash"
gnome-terminal -- bash -c "sleep 5; $SCRIPT_DIR/3e.sh; exec bash"
gnome-terminal -- bash -c "sleep 5; $SCRIPT_DIR/4r.sh; exec bash"
gnome-terminal -- bash -c "sleep 10; $SCRIPT_DIR/5t.sh; exec bash"
gnome-terminal -- bash -c "sleep 10; $SCRIPT_DIR/6y.sh; exec bash"


