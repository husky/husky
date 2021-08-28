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

#include "husky_base/husky_diagnostics.h"
#include <algorithm>
#include <limits>

namespace
{
  const int UNDERVOLT_ERROR = 18;
  const int UNDERVOLT_WARN = 19;
  const int OVERVOLT_ERROR = 30;
  const int OVERVOLT_WARN = 29;
  const int DRIVER_OVERTEMP_ERROR = 50;
  const int DRIVER_OVERTEMP_WARN = 30;
  const int MOTOR_OVERTEMP_ERROR = 80;
  const int MOTOR_OVERTEMP_WARN = 70;
  const double LOWPOWER_ERROR = 0.2;
  const double LOWPOWER_WARN = 0.3;
  const int CONTROLFREQ_WARN = 90;
  const unsigned int SAFETY_TIMEOUT = 0x1;
  const unsigned int SAFETY_LOCKOUT = 0x2;
  const unsigned int SAFETY_ESTOP = 0x8;
  const unsigned int SAFETY_CCI = 0x10;
  const unsigned int SAFETY_PSU = 0x20;
  const unsigned int SAFETY_CURRENT = 0x40;
  const unsigned int SAFETY_WARN = (SAFETY_TIMEOUT | SAFETY_CCI | SAFETY_PSU);
  const unsigned int SAFETY_ERROR = (SAFETY_LOCKOUT | SAFETY_ESTOP | SAFETY_CURRENT);
}  // namespace

namespace husky_base
{

  template<>
  HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus>::HuskyHardwareDiagnosticTask(husky_msgs::HuskyStatus &msg)
    :
    DiagnosticTask("system_status"),
    msg_(msg)
  { }

  template<>
  void HuskyHardwareDiagnosticTask<clearpath::DataSystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSystemStatus>::Ptr &system_status)
  {
    msg_.uptime = system_status->getUptime();

    msg_.battery_voltage = system_status->getVoltage(0);
    msg_.left_driver_voltage = system_status->getVoltage(1);
    msg_.right_driver_voltage = system_status->getVoltage(2);

    msg_.mcu_and_user_port_current = system_status->getCurrent(0);
    msg_.left_driver_current = system_status->getCurrent(1);
    msg_.right_driver_current = system_status->getCurrent(2);

    msg_.left_driver_temp = system_status->getTemperature(0);
    msg_.right_driver_temp = system_status->getTemperature(1);
    msg_.left_motor_temp = system_status->getTemperature(2);
    msg_.right_motor_temp = system_status->getTemperature(3);

    stat.add("Uptime", msg_.uptime);

    stat.add("Battery Voltage", msg_.battery_voltage);
    stat.add("Left Motor Driver Voltage", msg_.left_driver_voltage);
    stat.add("Right Motor Driver Voltage", msg_.right_driver_voltage);

    stat.add("MCU and User Port Current", msg_.mcu_and_user_port_current);
    stat.add("Left Motor Driver Current", msg_.left_driver_current);
    stat.add("Right Motor Driver Current", msg_.right_driver_current);

    stat.add("Left Motor Driver Temp (C)", msg_.left_driver_temp);
    stat.add("Right Motor Driver Temp (C)", msg_.right_driver_temp);
    stat.add("Left Motor Temp (C)", msg_.left_motor_temp);
    stat.add("Right Motor Temp (C)", msg_.right_motor_temp);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "System Status OK");
    if (msg_.battery_voltage > OVERVOLT_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery voltage too high");
    }
    else if (msg_.battery_voltage > OVERVOLT_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main battery voltage too high");
    }
    else if (msg_.battery_voltage < UNDERVOLT_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Main battery voltage too low");
    }
    else if (msg_.battery_voltage < UNDERVOLT_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Main battery voltage too low");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Voltage OK");
    }

    if (std::max(msg_.left_driver_temp, msg_.right_driver_temp) > DRIVER_OVERTEMP_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motor drivers too hot");
    }
    else if (std::max(msg_.left_driver_temp, msg_.right_driver_temp) > DRIVER_OVERTEMP_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motor drivers too hot");
    }
    else if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > MOTOR_OVERTEMP_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Motors too hot");
    }
    else if (std::max(msg_.left_motor_temp, msg_.right_motor_temp) > MOTOR_OVERTEMP_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Motors too hot");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature OK");
    }
  }

  template<>
  HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem>::HuskyHardwareDiagnosticTask(husky_msgs::HuskyStatus &msg)
    :
    DiagnosticTask("power_status"),
    msg_(msg)
  { }

  template<>
  void HuskyHardwareDiagnosticTask<clearpath::DataPowerSystem>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataPowerSystem>::Ptr &power_status)
  {
    msg_.charge_estimate = power_status->getChargeEstimate(0);
    msg_.capacity_estimate = power_status->getCapacityEstimate(0);

    stat.add("Charge (%)", msg_.charge_estimate);
    stat.add("Battery Capacity (Wh)", msg_.capacity_estimate);

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Power System OK");
    if (msg_.charge_estimate < LOWPOWER_ERROR)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Low power");
    }
    else if (msg_.charge_estimate < LOWPOWER_WARN)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Low power");
    }
    else
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::OK, "Charge OK");
    }
  }

  template<>
  HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::HuskyHardwareDiagnosticTask(
    husky_msgs::HuskyStatus &msg)
    :
    DiagnosticTask("safety_status"),
    msg_(msg)
  { }

  template<>
  void HuskyHardwareDiagnosticTask<clearpath::DataSafetySystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    horizon_legacy::Channel<clearpath::DataSafetySystemStatus>::Ptr &safety_status)
  {
    uint16_t flags = safety_status->getFlags();
    msg_.timeout = (flags & SAFETY_TIMEOUT) > 0;
    msg_.lockout = (flags & SAFETY_LOCKOUT) > 0;
    msg_.e_stop = (flags & SAFETY_ESTOP) > 0;
    msg_.ros_pause = (flags & SAFETY_CCI) > 0;
    msg_.no_battery = (flags & SAFETY_PSU) > 0;
    msg_.current_limit = (flags & SAFETY_CURRENT) > 0;

    stat.add("Timeout", static_cast<bool>(msg_.timeout));
    stat.add("Lockout", static_cast<bool>(msg_.lockout));
    stat.add("Emergency Stop", static_cast<bool>(msg_.e_stop));
    stat.add("ROS Pause", static_cast<bool>(msg_.ros_pause));
    stat.add("No battery", static_cast<bool>(msg_.no_battery));
    stat.add("Current limit", static_cast<bool>(msg_.current_limit));

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Safety System OK");
    if ((flags & SAFETY_ERROR) > 0)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::ERROR, "Safety System Error");
    }
    else if ((flags & SAFETY_WARN) > 0)
    {
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, "Safety System Warning");
    }
  }

  HuskySoftwareDiagnosticTask::HuskySoftwareDiagnosticTask(husky_msgs::HuskyStatus &msg, double target_control_freq)
    :
    DiagnosticTask("software_status"),
    msg_(msg),
    target_control_freq_(target_control_freq)
  {
    reset();
  }

  void HuskySoftwareDiagnosticTask::updateControlFrequency(double frequency)
  {
    // Keep minimum observed frequency for diagnostics purposes
    control_freq_ = std::min(control_freq_, frequency);
  }

  void HuskySoftwareDiagnosticTask::run(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    msg_.ros_control_loop_freq = control_freq_;
    stat.add("ROS Control Loop Frequency", msg_.ros_control_loop_freq);

    double margin = control_freq_ / target_control_freq_ * 100;

    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Software OK");
    if (margin < CONTROLFREQ_WARN)
    {
      std::ostringstream message;
      message << "Control loop executing " << 100 - static_cast<int>(margin) << "% slower than desired";
      stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN, message.str());
    }

    reset();
  }

  void HuskySoftwareDiagnosticTask::reset()
  {
    control_freq_ = std::numeric_limits<double>::infinity();
    target_control_freq_ = 0;
  }
}  // namespace husky_base
