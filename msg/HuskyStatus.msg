Header header

# MCU Uptime, in ms
uint32 uptime

# ROS Control loop frequency (PC-side)
float64 ros_control_loop_freq

# Voltage of platform components, in volts
float64 bus_voltage
float64 left_driver_voltage
float64 right_driver_voltage

# Component temperatures, in C
float64 left_driver_temp
float64 right_driver_temp
float64 left_motor_temp
float64 right_motor_temp

# Battery capacity (Wh) and charge (%) estimate
uint16 capacity_estimate
float64 charge_estimate

# Mapped to conditions below
uint16 flags

# Husky error/stop conditions
bool timeout
bool lockout
bool e_stop
bool ros_pause
bool no_battery
bool current_limit
