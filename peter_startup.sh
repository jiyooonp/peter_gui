#!/bin/bash

SETUP_BASH=~/Documents/iowa_ws/devel/setup.bash
ROBOT_IP="$1"

# Check if SETUP_BASH exists
if [ ! -f "$SETUP_BASH" ]; then
  echo "Setup bash file path is incorrect!"
  exit 1
fi

# Source the setup.bash
source "$SETUP_BASH"

# Run roslaunch in a new terminal and continue with the script
gnome-terminal -- bash -c "source $SETUP_BASH; roslaunch perception_refactor joy.launch robot_ip:=$ROBOT_IP; exec bash" &

# Run teleop.launch in a new terminal
gnome-terminal -- bash -c "source $SETUP_BASH; roslaunch perception_refactor teleop.launch; exec bash" &

# Run sw.launch in another new terminal
gnome-terminal -- bash -c "source $SETUP_BASH; roslaunch perception_refactor sw.launch; exec bash" &
