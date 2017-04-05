husky
=====

For Husky instructions and tutorials, please see [Robots/Husky](http://wiki.ros.org/Robots/Husky).

REAL ROBOT INSTRUCTIONS:

To use the Clearpath Robotics' Dual UR5 Husky, make sure you have your environment variables correctly set up. The correct setup (to be added in your real Husky's /etc/ros/setup.bash file (if you're using sim it will be your ~/.bashrc)) is listed below.

The correct way to bring up the arms and the moveit suite is to first bring up the arm drivers (ur_modern_driver ROS package), the planning execution node (which brings up move_group) and then RViz on the client computer which will interface to the robot. For the model to show up correctly you should have all drivers publishing on the robot, such as the flir_ptu, the arm drivers, the robotiq grippers, and the force torque. They should all be publishing their joint states from the drivers and then the model will show up neatly.

The commands to use a simple interface with the gripper:

1) Double check the gripper drivers are running

     rostopic list | grep gripper

2) Launch the simple gripper interface

     rosrun robotiq_s_model_control SModelSimpleController.py

3) For more complex control, see this  
     http://wiki.ros.org/robotiq/Tutorials/Control%20of%20an%20S-Model%20Gripper%20using%20the%20Modbus%20TCP%20Protocol

How to launch the planning execution node for the arms (while ssh'd inside robot)
  
  1) Launch the planning execution:
  
       roslaunch husky_dual_ur5_moveit_config husky_dual_ur5_planning_execution.launch real:=true

How to launch the arm drivers:

  1) Launch the drivers for left and right arms (after the arms have powered on and booted)
  
       roslaunch /etc/ros/indigo/husky-core.d/left_ur5.launch
      roslaunch /etc/ros/indigo/husky-core.d/right_ur5.launch

How to launch the RViz viewer on your computer:

  1) Set up your ROS_MASTER_URI and ROS_IP:
  
      export ROS_MASTER_URI=http://192.168.1.11:11311
      export ROS_IP=your_computers_lan_ip
  
  2) Download the Dual UR5 Husky code and install on your computer:
  
      mkdir -p husky_ws/src
      cd husky_ws/src && catkin_init_workspace
      git clone https://github.com/husky/husky
      cd husky && git checkout dual_ur5_husky
      cd ~/husky_ws && catkin_make install
      source ~/husky_ws/devel/setup.bash
  
  3) Run the moveit_rviz.launch file:
  
     roslaunch husky_dual_ur5_moveit_config moveit_rviz.launch config:=true
  
 How to read data from the force torques:
 
 1) Check the sensors are running
 
     rostopic list | grep force_torque
 
 2) Echo the topic
 
     rostopic echo husky_left_gripper/robotiq_force_torque_sensor
