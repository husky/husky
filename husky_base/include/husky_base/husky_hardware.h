/**
*
*  \author     Paul Bovbel <pbovbel@clearpathrobotics.com>
*  \copyright  Copyright (c) 2014-2015, Clearpath Robotics, Inc.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of Clearpath Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to code@clearpathrobotics.com
*
*/

#ifndef HUSKY_BASE_HUSKY_HARDWARE_H
#define HUSKY_BASE_HUSKY_HARDWARE_H

#include "husky_base/husky_diagnostics.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "hardware_interface/joint_state_interface.h"
#include "hardware_interface/joint_command_interface.h"
#include "hardware_interface/robot_hw.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "husky_msgs/HuskyStatus.h"
#include <string>

namespace husky_base
{

/**
* Class representing Husky hardware, allows for ros_control to modify internal state via joint interfaces
*/
class HuskyHardware :
  public hardware_interface::RobotHW
{
public:
  HuskyHardware(ros::NodeHandle nh, ros::NodeHandle private_nh, double target_control_freq);

  void updateJointsFromHardware();

  void writeCommandsToHardware();

  void updateDiagnostics();

  void reportLoopDuration(const ros::Duration &duration);

private:
  void initializeDiagnostics();

  void resetTravelOffset();

  void registerControlInterfaces();

  double linearToAngular(const double &travel) const;

  double angularToLinear(const double &angle) const;

  void limitDifferentialSpeed(double &travel_speed_left, double &travel_speed_right);

  ros::NodeHandle nh_, private_nh_;

  // ROS Control interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  // Diagnostics
  ros::Publisher diagnostic_publisher_;
  husky_msgs::HuskyStatus husky_status_msg_;
  diagnostic_updater::Updater diagnostic_updater_;
  HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus> system_status_task_;
  HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem> power_status_task_;
  HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus> safety_status_task_;
  HuskySoftwareDiagnosticTask software_status_task_;

  // ROS Parameters
  double wheel_diameter_, max_accel_, max_speed_;

  double polling_timeout_;

  /**
  * Joint structure that is hooked to ros_control's InterfaceManager, to allow control via diff_drive_controller
  */
  struct Joint
  {
    double position;
    double position_offset;
    double velocity;
    double effort;
    double velocity_command;

    Joint() :
      position(0), velocity(0), effort(0), velocity_command(0)
    { }
  }
  joints_[4];
};

}  // namespace husky_base
#endif  // HUSKY_BASE_HUSKY_HARDWARE_H
