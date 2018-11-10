# tractor_sim
Package for simulating the Olin Robotics Lab Autonomous Tractor

Installation:

`cd ~/catkin_ws/src`

`git clone https://github.com/AmyPhung/tractor_sim.git`

`cd ..`

`rosdep install -iry --from-paths src`

`cd ~/catkin_ws/`

`catkin_make`

Usage:

`cd ~/catkin_ws/src/tractor_sim`

To run simulation:

`roslaunch tractor_sim_gazebo tractor_sim.launch`

To run rviz:

`roslaunch tractor_sim_description tractor_sim_rviz_amcl.launch`

To run teleop (following https://github.com/olinrobotics/gravl/wiki/Kubo:-Overview):

`roslaunch gravl teleop.launch`

`rosrun gravl DriveState DriveState.cpp`

To test code:

`rosrun gravl yourscript.py` (ex: `rosrun gravl LidarFollower.py`)
