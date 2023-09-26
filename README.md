## Realsense camera
roslaunch realsense2_camera rs_aligned_depth.launch  filters:=pointcloud

rviz

## Planner node
rosrun perception_refactor planner_node.py

## Perception 
rosrun perception_refactor perception.py 
rosrun perception_refactor visual_servo_node.py

## Xarm
ping 192.168.1.196
go to 192.168.1.196 and enable robot

roslaunch visual_servo teleop.launch
roslaunch amiga_xarm xarm_joystick.launch
