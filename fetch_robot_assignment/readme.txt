NOTES FOR TEAM:
- You need the fetch_gazebo package: https://github.com/ZebraDevs/fetch_gazebo
- The first command just loads up Rviz preconfigured for the sensors in the fetch bot
- the two fetch_gazebo packages load in a fetch robot and and empty world (1st option) or simple room (2nd option)
- the simulation I made is the current main simulation that I was planning on using for the assignment

1. In terminal 1: roscore
2. In terminal 2: roslaunch fetch_robot_assignment simulation.launch
3. In terminal 3: roslaunch fetch_moveit_config  move_group.launch
4. In terminal 4: roslaunch fetch_robot_assignment main.launch

run simulation I made
roslaunch fetch_robot_assignment simulation.launch

run fetch robot gazebo
roslaunch fetch_gazebo simulation.launch

run fetch robot gazebo playground
roslaunch fetch_gazebo playground.launch

NEED TO RUN THIS TO MAKE ARM MOVE AT THIS POINT - this is not in the repo
roslaunch fetch_moveit_config  move_group.launch

TO DO:
YOU ARE CONVERTING THE VISUAL SERVOING TO IK MOVEMENTS
