* As of Feb 17th, 2015, the Husky UR5 has only been tested in simulation and not on real hardware.

=======================================
Installing The Husky UR5 from Source
=======================================

1. Clone these repositories and put them in your source directory, then catkin_make:

    git clone https://github.com/husky/husky_description

    git clone https://github.com/husky/husky_simulator

    git clone https://github.com/husky/husky_ur5_moveit_config

    mv robotiq grizzly_ur10 ~/catkin_ws/your_source_directory

    cd ~/catkin_ws

    catkin_make

Everything should compile smoothly (provided you have all of the debs installed. if not, install all of them until the compiler errors go away)

=======================================
Installing The Husky UR5 from debs
=======================================

When available on http://www.ros.org/debbuild/indigo.html,

   sudo apt-get install ros-indigo-husky-desktop

==============================
Usage with real Hardware (Not tested yet)
==============================

There are launch files available to bringup a real Grizzly + UR10 robot.
Don't forget to source the correct setup shell files and use a new terminal for each command!

To bring up the real robot, run:

    roslaunch grizzly_ur10_bringup grizzly_ur10_bringup.launch robot_ip:=IP_OF_THE_ROBOT     [reverse_port:=REVERSE_PORT]

A simple test script that moves the robot to predefined positions can be executed like this:

    rosrun ur_driver test_move.py

CAUTION:
Remember that you should always have your hands on the big red button in case there is something in the way or anything unexpected happens.

==============================
MoveIt! with real Hardware (Not tested yet)
==============================

Additionally, you can use MoveIt! to control the robot.
There exist MoveIt! configuration packages for both robots.

For setting up the MoveIt! nodes to allow motion planning run:

    roslaunch grizzly_ur10_moveit_config grizzly_ur10_moveit_planning_execution.launch

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

    roslaunch grizzly_ur10_moveit_config moveit_rviz.launch config:=true

As MoveIt! seems to have difficulties with finding plans for the UR with full joint limits [-2pi, 2pi], there is a joint_limited version using joint limits restricted to [-pi,pi]. In order to use this joint limited version, simply use the launch file arguments 'limited', i.e.:

    roslaunch grizzly_ur10_bringup grizzly_ur10_bringup.launch limited:=true robot_ip:=IP_OF_THE_ROBOT [reverse_port:=REVERSE_PORT]

    roslaunch grizzly_ur10_moveit_config grizzly_ur10_moveit_planning_execution.launch limited:=true

    roslaunch grizzly_ur10_moveit_config moveit_rviz.launch config:=true


===============================
Usage with Gazebo Simulation
===============================

There are launch files available to bringup a simulated robot.
In the following the commands for the UR5 are given. For the UR10, simply replace the prefix accordingly.

Don't forget to source the correct setup shell files and use a new terminal for each command!

To bring up the simulated robot in Gazebo, run:

    roslaunch husky_gazebo husky_ur5.launch

===============================
MoveIt! with a simulated robot
===============================

Have the Gazebo simulation up and running: 

    roslaunch husky_gazebo husky_ur5.launch

Start the trajectory execution for simulation. This will allow the controllers to move the robot and bring up move_group which accepts PlanningRequest messages.

    roslaunch husky_ur5_moveit_config husky_ur5_planning_execution.launch sim:=true

Start RViz which will provide an interface to move around the arm.

    roslaunch husky_ur5_moveit_config moveit_rviz.launch config:=true

Make sure you set the planning request to be: ur5_arm so you can control the arm. It will also have interactive markers around it. hit the "Plan and execute" button once you have selected a nice location for the arm, and by nice it means a place where there is no red. 

It should have a response time of less than 0.5s.






