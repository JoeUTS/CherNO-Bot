NOTES FOR TEAM:
- You need the fetch_gazebo package: https://github.com/ZebraDevs/fetch_gazebo
- The first command just loads up Rviz preconfigured for the sensors in the fetch bot
- the two fetch_gazebo packages load in a fetch robot and and empty world (1st option) or simple room (2nd option)
- the simulation I made is the current main simulation that I was planning on using for the assignment



Run RVIZ with config
rosrun rviz rviz -d /home/joseph/catkin_ws/src/fetch_robot_assignment/config/fetch_assignment.rviz

run fetch robot gazebo
roslaunch fetch_gazebo simulation.launch

run fetch robot gazebo playground
roslaunch fetch_gazebo playground.launch

run simulation I made
roslaunch fetch_robot_assignment simulation.launch

NEED TO RUN THIS TO MAKE ARM MOVE AT THIS POINT - this is not in the repo
roslaunch fetch_moveit_config  move_group.launch

TO DO:
You are currently trying to get the above file to launch with the simulation.launch file

YOU ARE TRYING TO GET THE VISUAL SERVOING WORKING
WE NEED TH Z VALUE BY FINDING THE TRANSFORM OF THE HEAD AND CALCULATING THE DISTANCE BETWEEN THIS AND THE OBJECTS. YOU NEED TO FIND THE TF NAMES VIA: rosrun tf view_frames.

