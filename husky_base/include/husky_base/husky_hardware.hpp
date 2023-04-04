#ifndef HUSKY_BASE__HUSKY_HARDWARE_HPP_
#define HUSKY_BASE__HUSKY_HARDWARE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"
#include "husky_msgs/msg/husky_status.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husky_base/horizon_legacy_wrapper.h"
#include "husky_base/husky_status.hpp"


using namespace std::chrono_literals;

namespace husky_base
{

class HuskyHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HuskyHardware)

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type start() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type stop() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type read() override;

  HARDWARE_INTERFACE_PUBLIC
  hardware_interface::return_type write() override;

private:
  void resetTravelOffset();
  double linearToAngular(const double &travel) const;
  double angularToLinear(const double &angle) const;
  void writeCommandsToHardware();
  void limitDifferentialSpeed(double &diff_speed_left, double &diff_speed_right);
  void updateJointsFromHardware();
  void readStatusFromHardware();
  uint8_t isLeft(const std::string &str);

  // ROS Parameters
  std::string serial_port_;
  double polling_timeout_;
  double wheel_diameter_, max_accel_, max_speed_;

  // Store the command for the robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_, hw_states_position_offset_, hw_states_velocity_;

  uint8_t left_cmd_joint_index_, right_cmd_joint_index_;

  std::shared_ptr<husky_status::HuskyStatus> status_node_;
  husky_msgs::msg::HuskyStatus status_msg_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE__HUSKY_HARDWARE_HPP_
