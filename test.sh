cat test.sh
!/usr/bin/env bash

konsole --noclose --new-tab -e  /bin/bash && source ~/iowa_ws/devel/setup.bash && roslaunch realsense2_camera rs_aligned_depth.launch  filters:=pointcloud 1! &
konsole --noclose --new-tab -e /bin/bash && source ~/iowa_ws/devel/setup.bash && rviz 2!&
konsole --noclose --new-tab -e /bin/bash && source ~/mrsd_teamD/bin/activate && source ~/iowa_ws/devel/setup.bash && rosrun fvd_ws planner_node.py 3!&
konsole --noclose --new-tab -e /bin/bash && source ~/mrsd_teamD/bin/activate && source ~/iowa_ws/devel/setup.bash && rosrun fvd_ws perception.py  4! &
konsole --noclose --new-tab -e /bin/bash && source ~/mrsd_teamD/bin/activate && source ~/iowa_ws/devel/setup.bash && rosrun fvd_ws visual_servo_node.py  5!  &
konsole --noclose --new-tab -e /bin/bash && source ~/iowa_ws/devel/setup.bash && roslaunch amiga_xarm xarm_joystick.launch 6! &
konsole --noclose --new-tab -e /bin/bash && source ~/iowa_ws/devel/setup.bash && roslaunch visual_servo teleop.launch 7! &