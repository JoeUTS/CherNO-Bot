- You need the fetch_gazebo package: https://github.com/ZebraDevs/fetch_gazebo


To run:
1. In terminal 1: roscore
2. In terminal 2: roslaunch fetch_robot_assignment simulation.launch
3. In terminal 3: roslaunch fetch_moveit_config  move_group.launch
*Once gazebo loaded and fetch robot in standby state*
4. In terminal 4: roslaunch fetch_robot_assignment main.launch
