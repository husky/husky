#!/usr/bin/env python
import roslib; roslib.load_manifest('husky_bringup')
import rospy

from clearpath_base.msg import SystemStatus, SafetyStatus, PowerStatus
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Constants
UNDERVOLT_ERROR = 18
UNDERVOLT_WARN = 19
OVERVOLT_ERROR = 30
OVERVOLT_WARN = 29
DRIVER_OVERTEMP_ERROR = 50
DRIVER_OVERTEMP_WARN = 30
MOTOR_OVERTEMP_ERROR = 80
MOTOR_OVERTEMP_WARN = 70
LOWPOWER_ERROR = 0.2
LOWPOWER_WARN = 0.3
SAFETY_TIMEOUT = 0x1
SAFETY_LOCKOUT = 0x2
SAFETY_ESTOP = 0x8
SAFETY_CCI = 0x10
SAFETY_PSU= 0x20
SAFETY_CURRENT = 0x40
SAFETY_WARN = (SAFETY_TIMEOUT | SAFETY_CCI | SAFETY_PSU)
SAFETY_ERROR = (SAFETY_LOCKOUT | SAFETY_ESTOP | SAFETY_CURRENT)

class HuskyDiagnostics():
    def __init__(self):
        rospy.init_node('husky_diagnostics')
        self.diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray)
        self.last_diagnostics_time = rospy.get_rostime()        
        rospy.Subscriber('husky/data/system_status', SystemStatus, self.HandleSystemStatus)
        rospy.Subscriber('husky/data/safety_status', SafetyStatus, self.HandleSafetyStatus)
        rospy.Subscriber('husky/data/power_status', PowerStatus, self.HandlePowerStatus)

        self.stat_estop = [] 
        self.stat_power = []
        self.stat_voltage = []
        self.stat_temp = []
        self.stat_uptime = []

        # Should publish: Estop, charge, voltage, current, temperatures

    def _publish(self, time):
        """ Publishes diagnostic data"""
        # Limit to 1 Hz
        if (time - self.last_diagnostics_time).to_sec() < 1.0:
            return
        self.last_diagnostics_time = time
        diag = DiagnosticArray()
        diag.header.stamp = time

        # E-Stop
        if self.stat_estop:
            diag.status.append(self.stat_estop)
        # Power
        if self.stat_power:
            diag.status.append(self.stat_power)
        # Voltage
        if self.stat_voltage:
            diag.status.append(self.stat_voltage)
        # Temperature
        if self.stat_temp:
            diag.status.append(self.stat_temp)
        # Uptime
        if self.stat_uptime:
            diag.status.append(self.stat_uptime)

        # Publish
        print diag
        self.diag_pub.publish(diag)

    def HandleSystemStatus(self,data):
        # Uptime
        self.stat_uptime = DiagnosticStatus(name="Uptime",level=DiagnosticStatus.OK, message="Robot Online")
        uptime = data.uptime
        self.stat_uptime.values = [KeyValue("Uptime (ms)",str(uptime))]
    
        # Bus voltage
        bus_volt = data.voltages[0]
        self.stat_voltage = DiagnosticStatus(name="Voltage",level=DiagnosticStatus.OK, message="Voltage OK")
        if bus_volt > OVERVOLT_ERROR:
            self.stat_voltage.level = DiagnosticStatus.ERROR
            self.stat_voltage.message = "Main bus voltage too high"
        elif bus_volt > OVERVOLT_WARN:
            self.stat_voltage.level = DiagnosticStatus.WARN
            self.stat_voltage.message = "Main bus voltage too high"
        elif bus_volt < UNDERVOLT_ERROR:
            self.stat_voltage.level = DiagnosticStatus.ERROR
            self.stat_voltage.message = "Main bus voltage too low"
        elif bus_volt < UNDERVOLT_WARN:
            self.stat_voltage.level = DiagnosticStatus.WARN
            self.stat_voltage.message = "Main bus voltage too low"

        self.stat_voltage.values = [KeyValue("Bus voltage (V)",str(bus_volt))]

        # Temperature
        self.stat_temp = DiagnosticStatus(name="Temperature",level=DiagnosticStatus.OK, message="OK")
        left_drv = data.temperatures[0]
        right_drv = data.temperatures[1]
        left_mot = data.temperatures[2]
        right_mot = data.temperatures[3]
        if max(left_drv,right_drv) > DRIVER_OVERTEMP_ERROR:
            self.stat_temp.level = DiagnosticStatus.ERROR
            self.stat_temp.message = "Motor drivers too hot"
        elif max(left_drv,right_drv) > DRIVER_OVERTEMP_WARN:
            self.stat_temp.level = DiagnosticStatus.WARN
            self.stat_temp.message = "Motor drivers too hot"
        if max(left_mot,right_mot) > MOTOR_OVERTEMP_ERROR:
            self.stat_temp.level = DiagnosticStatus.ERROR
            self.stat_temp.message = "Motors too hot"
        elif max(left_mot,right_mot) > MOTOR_OVERTEMP_WARN:
            self.stat_temp.level = DiagnosticStatus.WARN
            self.stat_temp.message = "Motors too hot"

        self.stat_temp.values = [KeyValue("Left motor driver (C)",str(left_drv)),
                                 KeyValue("Right motor driver (C)",str(right_drv)),
                                 KeyValue("Left motor (C)",str(left_mot)),
                                 KeyValue("Right motor (C)",str(right_mot))]

        # Publish
        self._publish(data.header.stamp)

    def HandleSafetyStatus(self,data):
        # Safety
        self.stat_estop = DiagnosticStatus(name="Safety System",level=DiagnosticStatus.OK,
                                           message="OK")
        flags = data.flags        

        if flags & SAFETY_ERROR:
            self.stat_estop.level = DiagnosticStatus.ERROR
            self.stat_estop.message = "Error"
        elif flags & SAFETY_WARN:
            self.stat_estop.level = DiagnosticStatus.WARN
            self.stat_estop.message = "Warning"

        self.stat_estop.values = [KeyValue("Timeout",str(flags & SAFETY_TIMEOUT > 0)),
                                 KeyValue("Lockout",str(flags & SAFETY_LOCKOUT > 0)), 
                                 KeyValue("Emergency Stop",str(flags & SAFETY_ESTOP > 0)), 
                                 KeyValue("ROS Pause",str(flags & SAFETY_CCI > 0)), 
                                 KeyValue("No battery",str(flags & SAFETY_PSU > 0)), 
                                 KeyValue("Current limit",str(flags & SAFETY_CURRENT > 0))]
        
        # Publish
        self._publish(data.header.stamp)

    def HandlePowerStatus(self,data):
        # Charge
        self.stat_power = DiagnosticStatus(name="Power System",level=DiagnosticStatus.OK, message="OK")
        charge = data.sources[0].charge
        capacity = data.sources[0].capacity
        present = data.sources[0].present
        bat_type = data.sources[0].description
        if charge < LOWPOWER_WARN:
            self.stat_power.level = DiagnosticStatus.WARN
            self.stat_power.message = "Low power"
        if charge < LOWPOWER_ERROR:
            self.stat_power.level = DiagnosticStatus.ERROR 
        self.stat_power.values = [KeyValue("Charge (%)",str(charge)),
                                 KeyValue("Battery Capacity (Wh)",str(capacity))]

        # Publish
        self._publish(data.header.stamp)

if __name__ == "__main__":
    obj = HuskyDiagnostics()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
