#include "husky_base/horizon_legacy/Message_data.h"
#include "husky_base/horizon_legacy/Number.h"
#include "husky_base/horizon_legacy/Transport.h"

#include <iostream>
#include <string>
#include <string.h>
#include <sstream>

using namespace std;

namespace clearpath
{

/**
* Macro which generates definitions of the Message constructors
* ExpectedLength is an expression valid within the constructor which gives the
* expected payload length of the message.  If the length reported in the message
* header does not match, and exception will be thrown.  If ExpectedLength is -1,
* the length check will be skipped.
* NB: Some Messages need to do some extra work in the constructor and don't use
*     this macro!
*/
#define MESSAGE_CONSTRUCTORS(MessageClass, ExpectedLength) \
MessageClass::MessageClass(void* input, size_t msg_len) : Message(input, msg_len) \
{ \
    if( ((ExpectedLength) >= 0) && ((ssize_t)getPayloadLength() != (ExpectedLength)) ) { \
        stringstream ss; \
        ss << "Bad payload length: actual="<<getPayloadLength(); \
        ss <<" vs. expected="<<(ExpectedLength); \
        throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH); \
    } \
} \
MessageClass::MessageClass(const MessageClass& other) : Message(other) {}


/**
* Macro which generates definitios of the Message convenience functions
* All message classes should use this macro to define these functions.
*/
#define MESSAGE_CONVENIENCE_FNS(MessageClass, DataMsgID) \
MessageClass* MessageClass::popNext() { \
    return dynamic_cast<MessageClass*>(Transport::instance().popNext(DataMsgID)); \
} \
\
MessageClass* MessageClass::waitNext(double timeout) { \
    return dynamic_cast<MessageClass*>(Transport::instance().waitNext(DataMsgID, timeout)); \
} \
\
MessageClass* MessageClass::getUpdate(double timeout) { \
    Transport::instance().flush(DataMsgID); \
    subscribe(0); \
    return dynamic_cast<MessageClass*>( \
            Transport::instance().waitNext(DataMsgID, timeout) ); \
}\
\
void MessageClass::subscribe(uint16_t freq) { \
    Request(DataMsgID-0x4000, freq).send(); \
} \
\
enum MessageTypes MessageClass::getTypeID() { \
    return DataMsgID; \
}

  MESSAGE_CONSTRUCTORS(DataAckermannOutput, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataAckermannOutput, DATA_ACKERMANN_SETPTS)

  double DataAckermannOutput::getSteering()
  {
    return btof(getPayloadPointer(STEERING), 2, 100);
  }

  double DataAckermannOutput::getThrottle()
  {
    return btof(getPayloadPointer(THROTTLE), 2, 100);
  }

  double DataAckermannOutput::getBrake()
  {
    return btof(getPayloadPointer(BRAKE), 2, 100);
  }

  ostream &DataAckermannOutput::printMessage(ostream &stream)
  {
    stream << "Ackermann Control" << endl;
    stream << "=================" << endl;
    stream << "Steering: " << getSteering() << endl;
    stream << "Throttle: " << getThrottle() << endl;
    stream << "Brake   : " << getBrake() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataDifferentialControl, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataDifferentialControl, DATA_DIFF_CTRL_CONSTS)

  double DataDifferentialControl::getLeftP()
  {
    return btof(getPayloadPointer(LEFT_P), 2, 100);
  }

  double DataDifferentialControl::getLeftI()
  {
    return btof(getPayloadPointer(LEFT_I), 2, 100);
  }

  double DataDifferentialControl::getLeftD()
  {
    return btof(getPayloadPointer(LEFT_D), 2, 100);
  }

  double DataDifferentialControl::getLeftFeedForward()
  {
    return btof(getPayloadPointer(LEFT_FEEDFWD), 2, 100);
  }

  double DataDifferentialControl::getLeftStiction()
  {
    return btof(getPayloadPointer(LEFT_STIC), 2, 100);
  }

  double DataDifferentialControl::getLeftIntegralLimit()
  {
    return btof(getPayloadPointer(LEFT_INT_LIM), 2, 100);
  }

  double DataDifferentialControl::getRightP()
  {
    return btof(getPayloadPointer(RIGHT_P), 2, 100);
  }

  double DataDifferentialControl::getRightI()
  {
    return btof(getPayloadPointer(RIGHT_I), 2, 100);
  }

  double DataDifferentialControl::getRightD()
  {
    return btof(getPayloadPointer(RIGHT_D), 2, 100);
  }

  double DataDifferentialControl::getRightFeedForward()
  {
    return btof(getPayloadPointer(RIGHT_FEEDFWD), 2, 100);
  }

  double DataDifferentialControl::getRightStiction()
  {
    return btof(getPayloadPointer(RIGHT_STIC), 2, 100);
  }

  double DataDifferentialControl::getRightIntegralLimit()
  {
    return btof(getPayloadPointer(RIGHT_INT_LIM), 2, 100);
  }

  ostream &DataDifferentialControl::printMessage(ostream &stream)
  {
    stream << "Differential Control Constant Data" << endl;
    stream << "==================================" << endl;
    stream << "Left P              : " << getLeftP() << endl;
    stream << "Left I              : " << getLeftI() << endl;
    stream << "Left D              : " << getLeftD() << endl;
    stream << "Left Feed Forward   : " << getLeftFeedForward() << endl;
    stream << "Left Stiction       : " << getLeftStiction() << endl;
    stream << "Left Integral Limit : " << getLeftIntegralLimit() << endl;
    stream << "Right P             : " << getRightP() << endl;
    stream << "Right I             : " << getRightI() << endl;
    stream << "Right D             : " << getRightD() << endl;
    stream << "Right Feed Forward  : " << getRightFeedForward() << endl;
    stream << "Right Stiction      : " << getRightStiction() << endl;
    stream << "Right Integral Limit: " << getRightIntegralLimit() << endl;
    return stream;
  }


  MESSAGE_CONSTRUCTORS(DataDifferentialOutput, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataDifferentialOutput, DATA_DIFF_WHEEL_SETPTS)

  double DataDifferentialOutput::getLeft()
  {
    return btof(getPayloadPointer(LEFT), 2, 100);
  }

  double DataDifferentialOutput::getRight()
  {
    return btof(getPayloadPointer(RIGHT), 2, 100);
  }

  ostream &DataDifferentialOutput::printMessage(ostream &stream)
  {
    stream << "Differential Output Data" << endl;
    stream << "========================" << endl;
    stream << "Left : " << getLeft() << endl;
    stream << "Right: " << getRight() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataDifferentialSpeed, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataDifferentialSpeed, DATA_DIFF_WHEEL_SPEEDS)

  double DataDifferentialSpeed::getLeftSpeed()
  {
    return btof(getPayloadPointer(LEFT_SPEED), 2, 100);
  }

  double DataDifferentialSpeed::getLeftAccel()
  {
    return btof(getPayloadPointer(LEFT_ACCEL), 2, 100);
  }

  double DataDifferentialSpeed::getRightSpeed()
  {
    return btof(getPayloadPointer(RIGHT_SPEED), 2, 100);
  }

  double DataDifferentialSpeed::getRightAccel()
  {
    return btof(getPayloadPointer(RIGHT_ACCEL), 2, 100);
  }

  ostream &DataDifferentialSpeed::printMessage(ostream &stream)
  {
    stream << "Differential Speed Data" << endl;
    stream << "=======================" << endl;
    stream << "Left Speed : " << getLeftSpeed() << endl;
    stream << "Left Accel : " << getLeftAccel() << endl;
    stream << "Right Speed: " << getRightSpeed() << endl;
    stream << "Right Accel: " << getRightAccel() << endl;
    return stream;
  }


  MESSAGE_CONSTRUCTORS(DataEcho, 0)

  MESSAGE_CONVENIENCE_FNS(DataEcho, DATA_ECHO)

  ostream &DataEcho::printMessage(ostream &stream)
  {
    stream << "Echo!";
    return stream;
  }


  DataEncoders::DataEncoders(void *input, size_t msg_len) : Message(input, msg_len)
  {
    if ((ssize_t) getPayloadLength() != (1 + getCount() * 6))
    {
      stringstream ss;
      ss << "Bad payload length: actual=" << getPayloadLength();
      ss << " vs. expected=" << (1 + getCount() * 6);
      throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH);
    }
    travels_offset = 1;
    speeds_offset = travels_offset + (getCount() * 4);
  }

  DataEncoders::DataEncoders(const DataEncoders &other) : Message(other)
  {
  }

  MESSAGE_CONVENIENCE_FNS(DataEncoders, DATA_ENCODER)

  uint8_t DataEncoders::getCount()
  {
    return *getPayloadPointer(0);
  }

  double DataEncoders::getTravel(uint8_t index)
  {
    return btof(getPayloadPointer(travels_offset + index * 4), 4, 1000);
  }

  double DataEncoders::getSpeed(uint8_t index)
  {
    return btof(getPayloadPointer(speeds_offset + index * 2), 2, 1000);
  }

  ostream &DataEncoders::printMessage(ostream &stream)
  {
    stream << "Encoder Data" << endl;
    stream << "============" << endl;
    stream << "Count   : " << (int) (getCount()) << endl;
    for (unsigned i = 0; i < getCount(); ++i)
    {
      stream << "Encoder " << i << ":" << endl;
      stream << "  Travel: " << getTravel(i) << endl;
      stream << "  Speed : " << getSpeed(i) << endl;
    }
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataEncodersRaw, (1 + getCount() * 4))

  MESSAGE_CONVENIENCE_FNS(DataEncodersRaw, DATA_ENCODER_RAW)

  uint8_t DataEncodersRaw::getCount()
  {
    return *getPayloadPointer(0);
  }

  int32_t DataEncodersRaw::getTicks(uint8_t inx)
  {
    return btoi(getPayloadPointer(1 + inx * 4), 4);
  }

  ostream &DataEncodersRaw::printMessage(ostream &stream)
  {
    stream << "Raw Encoder Data" << endl;
    stream << "================" << endl;
    for (int i = 0; i < getCount(); ++i)
    {
      stream << "Encoder " << i << ": " << getTicks(i) << endl;
    }
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataFirmwareInfo, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataFirmwareInfo, DATA_FIRMWARE_INFO)

  uint8_t DataFirmwareInfo::getMajorFirmwareVersion()
  {
    return *getPayloadPointer(MAJOR_FIRM_VER);
  }

  uint8_t DataFirmwareInfo::getMinorFirmwareVersion()
  {
    return *getPayloadPointer(MINOR_FIRM_VER);
  }

  uint8_t DataFirmwareInfo::getMajorProtocolVersion()
  {
    return *getPayloadPointer(MAJOR_PROTO_VER);
  }

  uint8_t DataFirmwareInfo::getMinorProtocolVersion()
  {
    return *getPayloadPointer(MINOR_PROTO_VER);
  }

  DataFirmwareInfo::WriteTime DataFirmwareInfo::getWriteTime()
  {
    return WriteTime(btou(getPayloadPointer(WRITE_TIME), 4));
  }

  ostream &DataFirmwareInfo::printMessage(ostream &stream)
  {
    stream << "Firmware Info" << endl;
    stream << "=============" << endl;
    stream << "Major firmware version: " << (int) getMajorFirmwareVersion() << endl;
    stream << "Minor firmware version: " << (int) getMinorFirmwareVersion() << endl;
    stream << "Major protocol version: " << (int) getMajorProtocolVersion() << endl;
    stream << "Minor protocol version: " << (int) getMinorProtocolVersion() << endl;
    WriteTime t = getWriteTime();
    stream << "Firmware write time   : ";
    stream << (2000 + t.year()) << "-" << (int) t.month() << "-" << (int) t.day() << " ";
    stream << (int) t.hour() << ":" << (int) t.minute() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataGear, 1)

  MESSAGE_CONVENIENCE_FNS(DataGear, DATA_GEAR_SETPT)

  uint8_t DataGear::getGear()
  {
    return getPayloadPointer()[0];
  }

  ostream &DataGear::printMessage(ostream &stream)
  {
    stream << "Gear" << endl;
    stream << "====" << endl;
    stream << "Gear: " << (int) getGear() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataMaxAcceleration, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataMaxAcceleration, DATA_MAX_ACCEL)

  double DataMaxAcceleration::getForwardMax()
  {
    return btof(getPayloadPointer(FORWARD_MAX), 2, 100);
  }

  double DataMaxAcceleration::getReverseMax()
  {
    return btof(getPayloadPointer(REVERSE_MAX), 2, 100);
  }

  ostream &DataMaxAcceleration::printMessage(ostream &stream)
  {
    stream << "Max Acceleration Data" << endl;
    stream << "=====================" << endl;
    stream << "Max Forward: " << getForwardMax() << endl;
    stream << "Max Reverse: " << getReverseMax() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataMaxSpeed, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataMaxSpeed, DATA_MAX_SPEED)

  double DataMaxSpeed::getForwardMax()
  {
    return btof(getPayloadPointer(FORWARD_MAX), 2, 100);
  }

  double DataMaxSpeed::getReverseMax()
  {
    return btof(getPayloadPointer(REVERSE_MAX), 2, 100);
  }

  ostream &DataMaxSpeed::printMessage(ostream &stream)
  {
    stream << "Max Speed Data" << endl;
    stream << "==============" << endl;
    stream << "Max Forward: " << getForwardMax() << endl;
    stream << "Max Reverse: " << getReverseMax() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformAcceleration, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataPlatformAcceleration, DATA_ACCEL)

  double DataPlatformAcceleration::getX()
  {
    return btof(getPayloadPointer(X), 2, 1000);
  }

  double DataPlatformAcceleration::getY()
  {
    return btof(getPayloadPointer(Y), 2, 1000);
  }

  double DataPlatformAcceleration::getZ()
  {
    return btof(getPayloadPointer(Z), 2, 1000);
  }

  ostream &DataPlatformAcceleration::printMessage(ostream &stream)
  {
    stream << "Platform Acceleration" << endl;
    stream << "=====================" << endl;
    stream << "X: " << getX() << endl;
    stream << "Y: " << getY() << endl;
    stream << "Z: " << getZ() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformInfo, (int) strlenModel() + 6)

  MESSAGE_CONVENIENCE_FNS(DataPlatformInfo, DATA_PLATFORM_INFO)

  uint8_t DataPlatformInfo::strlenModel()
  {
    return *getPayloadPointer();
  }

  string DataPlatformInfo::getModel()
  {
    char buf[256];
    // NB: cpy_len cannot be larger than 255 (one byte field!)
    size_t cpy_len = strlenModel();
    memcpy(buf, getPayloadPointer(1), cpy_len);
    buf[cpy_len] = '\0';

    return string(buf);
  }

  uint8_t DataPlatformInfo::getRevision()
  {
    char offset = strlenModel() + 1;
    return *getPayloadPointer(offset);
  }

  uint32_t DataPlatformInfo::getSerial()
  {
    char offset = strlenModel() + 2;
    return btou(getPayloadPointer(offset), 4);
  }

  std::ostream &DataPlatformInfo::printMessage(std::ostream &stream)
  {
    stream << "Platform Info" << endl;
    stream << "=============" << endl;
    stream << "Model   : " << getModel() << endl;
    stream << "Revision: " << (int) (getRevision()) << endl;
    stream << "Serial  : " << getSerial() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformName, (int) (*getPayloadPointer()) + 1)

  MESSAGE_CONVENIENCE_FNS(DataPlatformName, DATA_PLATFORM_NAME)

  string DataPlatformName::getName()
  {
    char buf[256];
    size_t cpy_len = *getPayloadPointer();
    memcpy(buf, getPayloadPointer(1), cpy_len);
    buf[cpy_len] = '\0';
    return string(buf);
  }

  std::ostream &DataPlatformName::printMessage(std::ostream &stream)
  {
    stream << "Platform Name" << endl;
    stream << "=============" << endl;
    stream << "Name: " << getName() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformMagnetometer, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataPlatformMagnetometer, DATA_MAGNETOMETER)

  double DataPlatformMagnetometer::getX()
  {
    return btof(getPayloadPointer(X), 2, 1000);
  }

  double DataPlatformMagnetometer::getY()
  {
    return btof(getPayloadPointer(Y), 2, 1000);
  }

  double DataPlatformMagnetometer::getZ()
  {
    return btof(getPayloadPointer(Z), 2, 1000);
  }

  ostream &DataPlatformMagnetometer::printMessage(ostream &stream)
  {
    stream << "PlatformMagnetometer Data" << endl;
    stream << "=================" << endl;
    stream << "X: " << getX() << endl;
    stream << "Y: " << getY() << endl;
    stream << "Z: " << getZ() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformOrientation, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataPlatformOrientation, DATA_ORIENT)

  double DataPlatformOrientation::getRoll()
  {
    return btof(getPayloadPointer(ROLL), 2, 1000);
  }

  double DataPlatformOrientation::getPitch()
  {
    return btof(getPayloadPointer(PITCH), 2, 1000);
  }

  double DataPlatformOrientation::getYaw()
  {
    return btof(getPayloadPointer(YAW), 2, 1000);
  }

  ostream &DataPlatformOrientation::printMessage(ostream &stream)
  {
    stream << "Platform Orientation" << endl;
    stream << "====================" << endl;
    stream << "Roll : " << getRoll() << endl;
    stream << "Pitch: " << getPitch() << endl;
    stream << "Yaw  : " << getYaw() << endl;

    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPlatformRotation, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataPlatformRotation, DATA_ROT_RATE)

  double DataPlatformRotation::getRollRate()
  {
    return btof(getPayloadPointer(ROLL_RATE), 2, 1000);
  }

  double DataPlatformRotation::getPitchRate()
  {
    return btof(getPayloadPointer(PITCH_RATE), 2, 1000);
  }

  double DataPlatformRotation::getYawRate()
  {
    return btof(getPayloadPointer(YAW_RATE), 2, 1000);
  }

  ostream &DataPlatformRotation::printMessage(ostream &stream)
  {
    stream << "Platform Rotationa Rate Data" << endl;
    stream << "============================" << endl;
    stream << "Roll : " << getRollRate() << endl;
    stream << "Pitch: " << getPitchRate() << endl;
    stream << "Yaw  : " << getYawRate() << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataPowerSystem, 1 + getBatteryCount() * 5)

  MESSAGE_CONVENIENCE_FNS(DataPowerSystem, DATA_POWER_SYSTEM)

  uint8_t DataPowerSystem::getBatteryCount()
  {
    return *getPayloadPointer(0);
  }

  double DataPowerSystem::getChargeEstimate(uint8_t battery)
  {
    int offset = 1 /* num batteries */
        + battery * 2;
    return btof(getPayloadPointer(offset), 2, 100);
  }

  int16_t DataPowerSystem::getCapacityEstimate(uint8_t battery)
  {
    int offset = 1 /* num batteries */
        + 2 * getBatteryCount() /*charge estimate data*/
        + battery * 2;
    return btoi(getPayloadPointer(offset), 2);
  }

  DataPowerSystem::BatteryDescription DataPowerSystem::getDescription(uint8_t battery)
  {
    int offset = 1 /* num batteries */
        + 4 * getBatteryCount() /* charge and capacity estimate data */
        + battery;
    return BatteryDescription(*getPayloadPointer(offset));
  }

  ostream &DataPowerSystem::printMessage(ostream &stream)
  {
    stream << "Power System Status Data" << endl;
    stream << "========================" << endl;
    int num_bat = getBatteryCount();
    stream << "Number of Batteries: " << num_bat << endl;
    for (int i = 0; i < num_bat; ++i)
    {
      stream << "Battery " << i << ":" << endl;
      stream << "  Charge Estimate  : " << getChargeEstimate(i) << endl;
      stream << "  Capacity Estimate: " << getCapacityEstimate(i) << endl;
      stream << "  Present          : " << (getDescription(0).isPresent() ? "yes" : "no") << endl;
      stream << "  In Use           : " << (getDescription(0).isInUse() ? "yes" : "no") << endl;
      stream << "  Type             : ";
      switch (getDescription(0).getType())
      {
        case BatteryDescription::EXTERNAL:
          stream << "External" << endl;
          break;
        case BatteryDescription::LEAD_ACID:
          stream << "Lead-Acid" << endl;
          break;
        case BatteryDescription::NIMH:
          stream << "NiMH" << endl;
          break;
        case BatteryDescription::GASOLINE:
          stream << "Internal Combustion Engine" << endl;
          break;
        default:
          stream << "Unknown Type" << endl;
          break;
      }
    }

    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataProcessorStatus, (1 + getProcessCount() * 2))

  MESSAGE_CONVENIENCE_FNS(DataProcessorStatus, DATA_PROC_STATUS)

  uint8_t DataProcessorStatus::getProcessCount()
  {
    return *getPayloadPointer();
  }

  int16_t DataProcessorStatus::getErrorCount(int process)
  {
    return btoi(getPayloadPointer(1 + process * 2), 2);
  }

  ostream &DataProcessorStatus::printMessage(ostream &stream)
  {
    stream << "Processor Status" << endl;
    stream << "================" << endl;
    stream << "Process Count   : " << (int) (getProcessCount()) << endl;
    for (unsigned int i = 0; i < getProcessCount(); ++i)
    {
      stream << "Process " << i << " Errors: " << getErrorCount(i) << endl;
    }
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRangefinders, (1 + getRangefinderCount() * 2))

  MESSAGE_CONVENIENCE_FNS(DataRangefinders, DATA_DISTANCE_DATA)

  uint8_t DataRangefinders::getRangefinderCount()
  {
    return *getPayloadPointer();
  }

  int16_t DataRangefinders::getDistance(int rangefinder)
  {
    return btoi(getPayloadPointer(1 + 2 * rangefinder), 2);
  }

  ostream &DataRangefinders::printMessage(ostream &stream)
  {
    stream << "Rangefinder Data" << endl;
    stream << "================" << endl;
    stream << "Rangefinder Count: " << (int) (getRangefinderCount()) << endl;
    for (unsigned int i = 0; i < getRangefinderCount(); ++i)
    {
      stream << "Distance " << i << "       : " << getDistance(i) << endl;
    }
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRangefinderTimings, (1 + getRangefinderCount() * 6))

  MESSAGE_CONVENIENCE_FNS(DataRangefinderTimings, DATA_DISTANCE_TIMING)

  uint8_t DataRangefinderTimings::getRangefinderCount()
  {
    return *getPayloadPointer();
  }

  int16_t DataRangefinderTimings::getDistance(int rangefinder)
  {
    return btoi(getPayloadPointer(1 + 2 * rangefinder), 2);
  }

  uint32_t DataRangefinderTimings::getAcquisitionTime(int rangefinder)
  {
    return btou(getPayloadPointer(1 + 2 * getRangefinderCount() + 4 * rangefinder), 4);
  }

  ostream &DataRangefinderTimings::printMessage(ostream &stream)
  {
    stream << "Rangefinder Timing Data" << endl;
    stream << "=======================" << endl;
    stream << "Rangefinder Count : " << (int) (getRangefinderCount()) << endl;
    for (unsigned int i = 0; i < getRangefinderCount(); ++i)
    {
      stream << "Rangefinder " << i << ":" << endl;
      stream << "  Distance        : " << getDistance(i) << endl;
      stream << "  Acquisition Time: " << getAcquisitionTime(i) << endl;
    }
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawAcceleration, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataRawAcceleration, DATA_ACCEL_RAW)

  uint16_t DataRawAcceleration::getX()
  {
    return btou(getPayloadPointer(X), 2);
  }

  uint16_t DataRawAcceleration::getY()
  {
    return btou(getPayloadPointer(Y), 2);
  }

  uint16_t DataRawAcceleration::getZ()
  {
    return btou(getPayloadPointer(Z), 2);
  }

  ostream &DataRawAcceleration::printMessage(ostream &stream)
  {
    stream << "Raw Acceleration Data" << endl;
    stream << "=====================" << endl;
    stream << "X: 0x" << hex << getX() << endl;
    stream << "Y: 0x" << getY() << endl;
    stream << "Z: 0x" << getZ() << dec << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawCurrent, (1 + getCurrentCount() * 2))

  MESSAGE_CONVENIENCE_FNS(DataRawCurrent, DATA_CURRENT_RAW)

  uint8_t DataRawCurrent::getCurrentCount()
  {
    return *getPayloadPointer();
  }

  uint16_t DataRawCurrent::getCurrent(int current)
  {
    return btou(getPayloadPointer(1 + current * 2), 2);
  }

  ostream &DataRawCurrent::printMessage(ostream &stream)
  {
    stream << "Raw Current Data" << endl;
    stream << "================" << endl;
    stream << hex;
    for (unsigned int i = 0; i < getCurrentCount(); ++i)
    {
      stream << "Current " << i << ": 0x" << getCurrent(i) << endl;
    }
    stream << dec;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawGyro, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataRawGyro, DATA_GYRO_RAW)

  uint16_t DataRawGyro::getRoll()
  {
    return btou(getPayloadPointer(ROLL), 2);
  }

  uint16_t DataRawGyro::getPitch()
  {
    return btou(getPayloadPointer(PITCH), 2);
  }

  uint16_t DataRawGyro::getYaw()
  {
    return btou(getPayloadPointer(YAW), 2);
  }

  ostream &DataRawGyro::printMessage(ostream &stream)
  {
    stream << "Raw Gyro Data" << endl;
    stream << "=============" << endl;
    stream << "Roll : 0x" << hex << getRoll() << endl;
    stream << "Pitch: 0x" << getPitch() << endl;
    stream << "Yaw  : 0x" << getYaw() << dec << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawMagnetometer, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataRawMagnetometer, DATA_MAGNETOMETER_RAW)

  uint16_t DataRawMagnetometer::getX()
  {
    return btou(getPayloadPointer(X), 2);
  }

  uint16_t DataRawMagnetometer::getY()
  {
    return btou(getPayloadPointer(Y), 2);
  }

  uint16_t DataRawMagnetometer::getZ()
  {
    return btou(getPayloadPointer(Z), 2);
  }

  ostream &DataRawMagnetometer::printMessage(ostream &stream)
  {
    stream << "Raw Magnetometer Data" << endl;
    stream << "=====================" << endl;
    stream << "X: 0x" << hex << getX() << endl;
    stream << "Y: 0x" << getY() << endl;
    stream << "Z: 0x" << getZ() << dec << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawOrientation, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataRawOrientation, DATA_ORIENT_RAW)

  uint16_t DataRawOrientation::getRoll()
  {
    return btou(getPayloadPointer(ROLL), 2);
  }

  uint16_t DataRawOrientation::getPitch()
  {
    return btou(getPayloadPointer(PITCH), 2);
  }

  uint16_t DataRawOrientation::getYaw()
  {
    return btou(getPayloadPointer(YAW), 2);
  }

  ostream &DataRawOrientation::printMessage(ostream &stream)
  {
    stream << "Raw Orientation Data" << endl;
    stream << "====================" << endl;
    stream << "Roll : 0x" << hex << getRoll() << endl;
    stream << "Pitch: 0x" << getPitch() << endl;
    stream << "Yaw  : 0x" << getYaw() << dec << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawTemperature, (1 + 2 * getTemperatureCount()))

  MESSAGE_CONVENIENCE_FNS(DataRawTemperature, DATA_TEMPERATURE_RAW)

  uint8_t DataRawTemperature::getTemperatureCount()
  {
    return *getPayloadPointer();
  }

  uint16_t DataRawTemperature::getTemperature(int temperature)
  {
    return btou(getPayloadPointer(1 + 2 * temperature), 2);
  }

  ostream &DataRawTemperature::printMessage(ostream &stream)
  {
    stream << "Raw Temperature Data" << endl;
    stream << "====================" << endl;
    stream << "Temperature Count: " << (int) (getTemperatureCount()) << endl;
    stream << hex;
    for (unsigned i = 0; i < getTemperatureCount(); ++i)
    {
      stream << "Temperature " << i << "    : 0x" << getTemperature(i) << endl;
    }
    stream << dec;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataRawVoltage, (1 + 2 * getVoltageCount()))

  MESSAGE_CONVENIENCE_FNS(DataRawVoltage, DATA_VOLTAGE_RAW)

  uint8_t DataRawVoltage::getVoltageCount()
  {
    return *getPayloadPointer();
  }

  uint16_t DataRawVoltage::getVoltage(int temperature)
  {
    return btou(getPayloadPointer(1 + 2 * temperature), 2);
  }

  ostream &DataRawVoltage::printMessage(ostream &stream)
  {
    stream << "Raw Voltage Data" << endl;
    stream << "================" << endl;
    stream << "Voltage Count: " << (int) (getVoltageCount()) << endl;
    stream << hex;
    for (unsigned i = 0; i < getVoltageCount(); ++i)
    {
      stream << "Voltage " << i << "    : 0x" << getVoltage(i) << endl;
    }
    stream << dec;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataSafetySystemStatus, 2)

  MESSAGE_CONVENIENCE_FNS(DataSafetySystemStatus, DATA_SAFETY_SYSTEM)

  uint16_t DataSafetySystemStatus::getFlags()
  {
    return btou(getPayloadPointer(), 2);
  }

  ostream &DataSafetySystemStatus::printMessage(ostream &stream)
  {
    stream << "Safety System Status Data" << endl;
    stream << "=========================" << endl;
    stream << "Flags: " << getFlags() << endl;
    return stream;
  }


  DataSystemStatus::DataSystemStatus(void *input, size_t msg_len) : Message(input, msg_len)
  {
    voltages_offset = 4;
    currents_offset = voltages_offset + 1 + getVoltagesCount() * 2;
    temperatures_offset = currents_offset + 1 + getCurrentsCount() * 2;

    size_t expect_len = (7 + 2 * getVoltagesCount() + 2 * getCurrentsCount() + 2 * getTemperaturesCount());
    if (getPayloadLength() != expect_len)
    {
      stringstream ss;
      ss << "Bad payload length: actual=" << getPayloadLength();
      ss << " vs. expected=" << expect_len;
      throw new MessageException(ss.str().c_str(), MessageException::INVALID_LENGTH);
    }
  }

  DataSystemStatus::DataSystemStatus(const DataSystemStatus &other) : Message(other)
  {
  }

  MESSAGE_CONVENIENCE_FNS(DataSystemStatus, DATA_SYSTEM_STATUS)

  uint32_t DataSystemStatus::getUptime()
  {
    return btou(getPayloadPointer(0), 4);
  }

  uint8_t DataSystemStatus::getVoltagesCount()
  {
    return *getPayloadPointer(voltages_offset);
  }

  double DataSystemStatus::getVoltage(uint8_t index)
  {
    return btof(getPayloadPointer(voltages_offset + 1 + (index * 2)), 2, 100);
  }

  uint8_t DataSystemStatus::getCurrentsCount()
  {
    return *getPayloadPointer(currents_offset);
  }

  double DataSystemStatus::getCurrent(uint8_t index)
  {
    return btof(getPayloadPointer(currents_offset + 1 + (index * 2)), 2, 100);
  }

  uint8_t DataSystemStatus::getTemperaturesCount()
  {
    return *getPayloadPointer(temperatures_offset);
  }

  double DataSystemStatus::getTemperature(uint8_t index)
  {
    return btof(getPayloadPointer(temperatures_offset + 1 + (index * 2)), 2, 100);
  }

  ostream &DataSystemStatus::printMessage(ostream &stream)
  {
    stream << "System Status" << endl;
    stream << "=============" << endl;
    stream << "Uptime           : " << getUptime() << endl;
    stream << "Voltage Count    : " << (int) (getVoltagesCount()) << endl;
    stream << "Voltages         : ";
    for (unsigned i = 0; i < getVoltagesCount(); ++i)
    {
      stream << getVoltage(i);
      if ((int) i != (getVoltagesCount() - 1)) { stream << ", "; }
    }
    stream << endl;
    stream << "Current Count    : " << (int) (getCurrentsCount()) << endl;
    stream << "Currents         : ";
    for (unsigned i = 0; i < getCurrentsCount(); ++i)
    {
      stream << getCurrent(i);
      if ((int) i != (getCurrentsCount() - 1)) { stream << ", "; }
    }
    stream << endl;
    stream << "Temperature Count: " << (int) (getTemperaturesCount()) << endl;
    stream << "Temperatures     : ";
    for (unsigned i = 0; i < getTemperaturesCount(); ++i)
    {
      stream << getTemperature(i);
      if ((int) i != (getTemperaturesCount() - 1)) { stream << ", "; }
    }
    stream << endl;
    return stream;
  }



  MESSAGE_CONSTRUCTORS(DataVelocity, PAYLOAD_LEN)

  MESSAGE_CONVENIENCE_FNS(DataVelocity, DATA_VELOCITY_SETPT)

  double DataVelocity::getTranslational()
  {
    return btof(getPayloadPointer(TRANS_VEL), 2, 100);
  }

  double DataVelocity::getRotational()
  {
    return btof(getPayloadPointer(ROTATIONAL), 2, 100);
  }

  double DataVelocity::getTransAccel()
  {
    return btof(getPayloadPointer(TRANS_ACCEL), 2, 100);
  }

  ostream &DataVelocity::printMessage(ostream &stream)
  {
    stream << "Velocity Setpoints" << endl;
    stream << "==================" << endl;
    stream << "Translational:" << getTranslational() << endl;
    stream << "Rotational:   " << getRotational() << endl;
    stream << "Trans Accel:  " << getTransAccel() << endl;
    return stream;
  }

} // namespace clearpath
