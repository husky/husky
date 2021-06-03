#ifndef HUSKY_BASE__HUSKY_BASE_HPP_
#define HUSKY_BASE__HUSKY_BASE_HPP_

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
#include "rclcpp/macros.hpp"


#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "husky_base/horizon_legacy_wrapper.h"

using namespace std::chrono_literals;


namespace husky_base
{

class HuskyBase
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HuskyBase);

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
  } joints_[4];

  // ROS Parameters
  std::string serial_port_;
  double wheel_diameter_, max_accel_, max_speed_;
  double polling_timeout_;
  
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_;
};

}  // namespace husky_base

#endif  // HUSKY_BASE__HUSKY_BASE_HPP_