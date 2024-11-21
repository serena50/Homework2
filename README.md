# Homework2
- Build the new package:
$ colcon build

- Source the setup files:
$ source install/setup.bash

- Launch the robot with:
$ ros2 launch iiwa_bringup iiwa.launch.py

N.B.: As soon as the gazebo opens, the simulation must be started within 5 seconds pushing the play button, otherwise it won't load the controllers.

- Connect to the container from another terminal:
$ ./docker_connect.sh 

Once the terminal is connected is possible :

N.B.: Before using any other terminal, source the setup files.

- Start ros2_kdl_nodeJS, in order to observe the trajectories in the Joint Space with:
$ ros2 run ros2_kdl_package ros2_kdl_nodeJS

It does the linear trajectory with trapezoidal velocity profile by default
while to choose the orher trajectory :

 -- linear trajectory with cubic velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeJS --ros-args -p path_type:=linear -p vel_profile:=cubic

 -- circular trajectory with trapezoidal velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeJS --ros-args -p path_type:=circular -p vel_profile:=trapezoidal

 -- linear trajectory with cubic velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeJS --ros-args -p path_type:=linear -p vel_profile:=cubic

- Start ros2_kdl_nodeOS, in order to observe the trajectories in the Operational Space with:
$ ros2 run ros2_kdl_package ros2_kdl_nodeOS

It does the linear trajectory with trapezoidal velocity profile by default
while to choose the orher trajectory :

 -- linear trajectory with cubic velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeOS --ros-args -p path_type:=linear -p vel_profile:=cubic

 -- circular trajectory with trapezoidal velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeOS --ros-args -p path_type:=circular -p vel_profile:=trapezoidal

 -- linear trajectory with cubic velocity profile--
$ ros2 run ros2_kdl_package ros2_kdl_nodeOS --ros-args -p path_type:=linear -p vel_profile:=cubic

N.B.: if the profile or the trajectory is written wrong, it does the default trajectory

- Open rqt to see the torque commands, on another terminal connected to the docker:
$ ros2 run rqt_plot rqt_plot /effort_controller/commands/data[0] /effort_controller/commands/data[1] /effort_controller/commands/data[2] /effort_controller/commands/data[3] /effort_controller/commands/data[4] /effort_controller/commands/data[5] /effort_controller/commands/data[6]
