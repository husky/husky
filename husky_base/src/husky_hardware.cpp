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

#include "husky_base/husky_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
  const uint8_t LEFT = 0, RIGHT = 1;
}

namespace husky_base
{
  static const std::string HW_NAME = "HuskyHardware";
  static const std::string LEFT_CMD_JOINT_NAME = "front_left_wheel_joint";
  static const std::string RIGHT_CMD_JOINT_NAME = "front_right_wheel_joint";

  /**
  * Get current encoder travel offsets from MCU and bias future encoder readings against them
  */
  void HuskyHardware::resetTravelOffset()
  {
    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc =
        horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
    if (enc)
    {
      for (auto i = 0u; i < hw_states_position_offset_.size(); i++)
      {
        hw_states_position_offset_[i] = linearToAngular(enc->getTravel(isLeft(info_.joints[i].name)));
      }
    }
    else
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME), "Could not get encoder data to calibrate travel offset");
    }
  }

  /**
  * Husky reports travel in metres, need radians for ros_control RobotHW
  */
  double HuskyHardware::linearToAngular(const double &travel) const
  {
    return (travel / wheel_diameter_ * 2.0f);
  }

  /**
  * RobotHW provides velocity command in rad/s, Husky needs m/s,
  */
  double HuskyHardware::angularToLinear(const double &angle) const
  {
    return (angle * wheel_diameter_ / 2.0f);
  }

  void HuskyHardware::writeCommandsToHardware()
  {
    double diff_speed_left = angularToLinear(hw_commands_[left_cmd_joint_index_]);
    double diff_speed_right = angularToLinear(hw_commands_[right_cmd_joint_index_]);

    limitDifferentialSpeed(diff_speed_left, diff_speed_right);

    horizon_legacy::controlSpeed(diff_speed_left, diff_speed_right, max_accel_, max_accel_);
  }

  void HuskyHardware::limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right)
  {
    double large_speed = std::max(std::abs(diff_speed_left), std::abs(diff_speed_right));

    if (large_speed > max_speed_)
    {
      diff_speed_left *= max_speed_ / large_speed;
      diff_speed_right *= max_speed_ / large_speed;
    }
  }


  /**
  * Pull latest speed and travel measurements from MCU, and store in joint structure for ros_control
  */
  void HuskyHardware::updateJointsFromHardware()
  {

    horizon_legacy::Channel<clearpath::DataEncoders>::Ptr enc =
      horizon_legacy::Channel<clearpath::DataEncoders>::requestData(polling_timeout_);
    if (enc)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger(HW_NAME),
        "Received linear distance information (L: %f, R: %f)",
        enc->getTravel(LEFT), enc->getTravel(RIGHT));

      for (auto i = 0u; i < hw_states_position_.size(); i++)
      {
        double delta = linearToAngular(enc->getTravel(isLeft(info_.joints[i].name)))
            - hw_states_position_[i] - hw_states_position_offset_[i];

        // detect suspiciously large readings, possibly from encoder rollover
        if (std::abs(delta) < 1.0f)
        {
          hw_states_position_[i] += delta;
        }
        else
        {
          // suspicious! drop this measurement and update the offset for subsequent readings
          hw_states_position_offset_[i] += delta;
          RCLCPP_WARN(
            rclcpp::get_logger(HW_NAME),"Dropping overflow measurement from encoder");
        }
      }
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(HW_NAME), "Could not get encoder data");
    }

    horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed =
      horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(polling_timeout_);
    if (speed)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger(HW_NAME),
        "Received linear speed information (L: %f, R: %f)",
        speed->getLeftSpeed(), speed->getRightSpeed());

      for (auto i = 0u; i < hw_states_velocity_.size(); i++)
      {
        if (isLeft(info_.joints[i].name) == LEFT)
        {
          hw_states_velocity_[i] = linearToAngular(speed->getLeftSpeed());
        }
        else
        { // assume RIGHT
          hw_states_velocity_[i] = linearToAngular(speed->getRightSpeed());
        }
      }
    }
        else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(HW_NAME), "Could not get speed data");
    }
  }

  /**
  * Determines if the joint is left or right based on the joint name
  */
  uint8_t HuskyHardware::isLeft(const std::string &str)
  {
    if (str.find("left") != std::string::npos)
    {
      return LEFT;
    }
    return RIGHT;
  }


hardware_interface::return_type HuskyHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Name: %s", info_.name.c_str());

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Number of Joints %u", info_.joints.size());

  hw_states_position_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_position_offset_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_states_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  wheel_diameter_ = std::stod(info_.hardware_parameters["wheel_diameter"]);
  max_accel_ = std::stod(info_.hardware_parameters["max_accel"]);
  max_speed_ = std::stod(info_.hardware_parameters["max_speed"]);
  polling_timeout_ = std::stod(info_.hardware_parameters["polling_timeout"]);

  serial_port_ = info_.hardware_parameters["serial_port"];

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Port: %s", serial_port_.c_str());
  horizon_legacy::connect(serial_port_);
  horizon_legacy::configureLimits(max_speed_, max_accel_);
  resetTravelOffset();

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // HuskyHardware has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger(HW_NAME),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> HuskyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_position_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_states_velocity_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HuskyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));

    // Detmerine which joints will be used for commands since Husky only has two motors
    if (info_.joints[i].name == LEFT_CMD_JOINT_NAME)
    {
      left_cmd_joint_index_ = i;
    }

    if (info_.joints[i].name == RIGHT_CMD_JOINT_NAME)
    {
      right_cmd_joint_index_ = i;
    }
  }

  return command_interfaces;
}

hardware_interface::return_type HuskyHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Starting ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_states_position_.size(); i++)
  {
    if (std::isnan(hw_states_position_[i]))
    {
      hw_states_position_[i] = 0;
      hw_states_position_offset_[i] = 0;
      hw_states_velocity_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "Stopping ...please wait...");

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger(HW_NAME), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::read()
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Reading from hardware");

  updateJointsFromHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully read!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type HuskyHardware::write()
{
  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Writing to hardware");

  writeCommandsToHardware();

  RCLCPP_DEBUG(rclcpp::get_logger(HW_NAME), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace husky_base

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  husky_base::HuskyHardware, hardware_interface::SystemInterface)
