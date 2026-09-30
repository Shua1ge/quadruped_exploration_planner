指令集：
pkill -INT -f '[l]aunch_exploration.sh'
pkill -INT -f '[r]os2 launch.*scan_planner'
pkill -INT -f '[g]z sim'
pkill -INT -f '/home/t1an/ros2_ws/scan_planner_ws/install/'****

ROS_DOMAIN_ID=43 ros2 daemon stop
sleep 1
ROS_DOMAIN_ID=43 ros2 daemon start
sleep 2
