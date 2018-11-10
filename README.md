# tractor_sim
Package for simulating the Olin Robotics Lab Autonomous Tractor

Installation:
`cd ~/your_catkin_ws/src`
`git clone https://github.com/AmyPhung/tractor_sim.git`
`cd ~/your_catkin_ws/src/tractor_sim`
`sudo rosdep install *`
`cd ~/your_catkin_ws/`
`catkin_make`

Usage:
`cd ~/your_catkin_ws/src/tractor_sim`
To run simulation:
`roslaunch tractor_sim_gazebo tractor_sim.launch`
To run rviz:
`roslaunch tractor_sim_description tractor_sim_rviz_amcl.launch`
To test code (following https://github.com/olinrobotics/gravl/wiki/Kubo:-Overview):
`roslaunch gravl teleop.launch`

