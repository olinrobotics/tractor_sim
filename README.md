# tractor_sim
Package for simulating the Olin Robotics Lab Autonomous Tractor. For further project information, see the GRAVL main repository wiki: ([link](https://github.com/olinrobotics/gravl/wiki))

## Setup
Dependencies:
+ [GRAVL](https://github.com/olinrobotics/gravl)
+ Follow setup instructions on the GRAVL wiki if you haven't already ([reference](https://github.com/olinrobotics/gravl/wiki))

Installation:
+ `cd ~/catkin_ws/src`
+ `git clone https://github.com/AmyPhung/tractor_sim.git`
+ `cd ..`
+ `rosdep install -iry --from-paths src`
+ `cd ~/catkin_ws/`
+ `catkin_make`

To use the models included in this repo, copy the contents of the folder to `~/.gazebo/models`
+ `cd ~/catkin_ws/src/tractor_sim/models`
+ `cp -R . ~/.gazebo/models`

## Usage
To run simulation:
+ `roslaunch gravl simulation.launch`

To run rviz:
+  `roslaunch tractor_sim_description tractor_sim_rviz_amcl.launch`

To test code:
+ `rosrun gravl yourscript.py` (ex: `rosrun gravl LidarFollower.py`)

If these throw errors, you can run indivisual pieces of the simulation to just test what you need.
To run simulation (without teleop or any code running)
+ `roslaunch tractor_sim_gazebo tractor_sim.launch`

To run teleop (following https://github.com/olinrobotics/gravl/wiki/Kubo:-Overview):
+ `roslaunch gravl teleop.launch`
+ `roslaunch gravl mainstate.launch`

To setup localization:
+ `roslaunch gravl localization.launch`


### Parameters
+ **tractor_delay** - either 0 or 1. If set to 1, then tractor will react instantaneously to commands. If set to 1, the tractor will simulate the delayed response time of the real tractor (Default: 1)
+ **max_acceleration** - maximum acceleration for the tractor Note: only works when tractor_delay is 0 (Default: 0.4)
+ **max_steering_angle_velocity** - maximum steering velocity Note: only works when tractor_delay is set to 0 (Default: 1)
