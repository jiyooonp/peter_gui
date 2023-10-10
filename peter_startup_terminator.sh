#!/bin/bash

SETUP_BASH="$HOME/Documents/iowa_ws/devel/setup.bash"
ROBOT_IP=$1

# Check if setup.bash exists
if [ ! -f $SETUP_BASH ]; then
  echo "Setup bash file path is incorrect!"
  exit 1
fi

# Source the setup.bash
source ${SETUP_BASH}

# Create the Terminator command with splits and custom commands
terminator_command="terminator"

# Add first pane (it will be the default one)
terminator_command+=" --command=\"bash -c 'source $SETUP_BASH; roslaunch perception_refactor teleop.launch; exec bash'\""

# Add horizontal split and second pane
terminator_command+=" --geometry=1920x1080 -H --command=\"bash -c 'source $SETUP_BASH; roslaunch perception_refactor sw.launch; exec bash'\""

# Add vertical split and third pane
terminator_command+=" -v --command=\"bash -c 'source $SETUP_BASH; roslaunch perception_refactor joy.launch; exec bash'\""

# Run the Terminator command
eval $terminator_command
