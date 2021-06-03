/**
*      _____
*     /  _  \
*    / _/ \  \
*   / / \_/   \
*  /  \_/  _   \  ___  _    ___   ___   ____   ____   ___   _____  _   _
*  \  / \_/ \  / /  _\| |  | __| / _ \ | ++ \ | ++ \ / _ \ |_   _|| | | |
*   \ \_/ \_/ /  | |  | |  | ++ | |_| || ++ / | ++_/| |_| |  | |  | +-+ |
*    \  \_/  /   | |_ | |_ | ++ |  _  || |\ \ | |   |  _  |  | |  | +-+ |
*     \_____/    \___/|___||___||_| |_||_| \_\|_|   |_| |_|  |_|  |_| |_|
*             ROBOTICS™
*
*  File: HorizonMessage_data.h
*  Desc: Provides Message subclasses corresponding to 'data' type messages
*  Auth: Iain Peet, Mike Purvis
*
*  Copyright (c) 2010, Clearpath Robotics, Inc.
*  All Rights Reserved
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
* Please send comments, questions, or patches to skynet@clearpathrobotics.com
*
*/


#ifndef CLEARPATH_MESSAGE_DATA_H
#define CLEARPATH_MESSAGE_DATA_H

#include <iostream>
#include <string>
#include <stdint.h>
#include "husky_base/horizon_legacy/Message.h"
#include "husky_base/horizon_legacy/Message_request.h"

namespace clearpath
{

  class DataAckermannOutput : public Message
  {
  public:
    enum payloadOffsets
    {
      STEERING = 0,
      THROTTLE = 2,
      BRAKE = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataAckermannOutput(void *input, size_t msg_len);

    DataAckermannOutput(const DataAckermannOutput &other);

    static DataAckermannOutput *popNext();

    static DataAckermannOutput *waitNext(double timeout = 0);

    static DataAckermannOutput *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq = 0);

    static enum MessageTypes getTypeID();

    double getSteering();

    double getThrottle();

    double getBrake();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataDifferentialControl : public Message
  {
  public:
    enum payloadOffsets
    {
      LEFT_P = 0,
      LEFT_I = 2,
      LEFT_D = 4,
      LEFT_FEEDFWD = 6,
      LEFT_STIC = 8,
      LEFT_INT_LIM = 10,
      RIGHT_P = 12,
      RIGHT_I = 14,
      RIGHT_D = 16,
      RIGHT_FEEDFWD = 18,
      RIGHT_STIC = 20,
      RIGHT_INT_LIM = 22,
      PAYLOAD_LEN = 24
    };

  public:
    DataDifferentialControl(void *input, size_t msg_len);

    DataDifferentialControl(const DataDifferentialControl &other);

    static DataDifferentialControl *popNext();

    static DataDifferentialControl *waitNext(double timeout = 0);

    static DataDifferentialControl *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq = 0);

    static enum MessageTypes getTypeID();

    double getLeftP();

    double getLeftI();

    double getLeftD();

    double getLeftFeedForward();

    double getLeftStiction();

    double getLeftIntegralLimit();

    double getRightP();

    double getRightI();

    double getRightD();

    double getRightFeedForward();

    double getRightStiction();

    double getRightIntegralLimit();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataDifferentialOutput : public Message
  {
  public:
    enum payloadOffsets
    {
      LEFT = 0,
      RIGHT = 2,
      PAYLOAD_LEN = 4
    };

  public:
    DataDifferentialOutput(void *input, size_t msg_len);

    DataDifferentialOutput(const DataDifferentialOutput &other);

    static DataDifferentialOutput *popNext();

    static DataDifferentialOutput *waitNext(double timeout = 0);

    static DataDifferentialOutput *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getLeft();

    double getRight();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataDifferentialSpeed : public Message
  {
  public:
    enum payloadOffsets
    {
      LEFT_SPEED = 0,
      RIGHT_SPEED = 2,
      LEFT_ACCEL = 4,
      RIGHT_ACCEL = 6,
      PAYLOAD_LEN = 8
    };

  public:
    DataDifferentialSpeed(void *input, size_t msg_len);

    DataDifferentialSpeed(const DataDifferentialSpeed &other);

    static DataDifferentialSpeed *popNext();

    static DataDifferentialSpeed *waitNext(double timeout = 0);

    static DataDifferentialSpeed *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getLeftSpeed();

    double getLeftAccel();

    double getRightSpeed();

    double getRightAccel();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataEcho : public Message
  {
  public:
    DataEcho(void *input, size_t msg_len);

    DataEcho(const DataEcho &other);

    static DataEcho *popNext();

    static DataEcho *waitNext(double timeout = 0);

    static DataEcho *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataEncoders : public Message
  {
  private:
    size_t travels_offset;
    size_t speeds_offset;

  public:
    DataEncoders(void *input, size_t msg_len);

    DataEncoders(const DataEncoders &other);

    static DataEncoders *popNext();

    static DataEncoders *waitNext(double timeout = 0);

    static DataEncoders *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getCount();

    double getTravel(uint8_t index);

    double getSpeed(uint8_t index);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataEncodersRaw : public Message
  {
  public:
    DataEncodersRaw(void *input, size_t pkt_len);

    DataEncodersRaw(const DataEncodersRaw &other);

    static DataEncodersRaw *popNext();

    static DataEncodersRaw *waitNext(double timeout = 0);

    static DataEncodersRaw *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getCount();

    int32_t getTicks(uint8_t inx);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataFirmwareInfo : public Message
  {
  public:
    enum payloadOffsets
    {
      MAJOR_FIRM_VER = 0,
      MINOR_FIRM_VER,
      MAJOR_PROTO_VER,
      MINOR_PROTO_VER,
      WRITE_TIME,
      PAYLOAD_LEN = 8
    };

    class WriteTime
    {
    public:
      uint32_t rawTime;
    public:
      WriteTime(uint32_t time) : rawTime(time)
      {
      }

      uint8_t minute()
      {
        return (rawTime) & 0x3f;
      }

      uint8_t hour()
      {
        return (rawTime >> 6) & 0x1f;
      }

      uint8_t day()
      {
        return (rawTime >> 11) & 0x3f;
      }

      uint8_t month()
      {
        return (rawTime >> 17) & 0x0f;
      }

      uint8_t year()
      {
        return (rawTime >> 21) & 0x7f;
      }
    };

  public:
    DataFirmwareInfo(void *input, size_t msg_len);

    DataFirmwareInfo(const DataFirmwareInfo &other);

    static DataFirmwareInfo *popNext();

    static DataFirmwareInfo *waitNext(double timeout = 0);

    static DataFirmwareInfo *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getMajorFirmwareVersion();

    uint8_t getMinorFirmwareVersion();

    uint8_t getMajorProtocolVersion();

    uint8_t getMinorProtocolVersion();

    WriteTime getWriteTime();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataGear : public Message
  {
  public:
    DataGear(void *input, size_t msg_len);

    DataGear(const DataGear &other);

    static DataGear *popNext();

    static DataGear *waitNext(double timeout = 0);

    static DataGear *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getGear();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataMaxAcceleration : public Message
  {
  public:
    enum payloadOffsets
    {
      FORWARD_MAX = 0,
      REVERSE_MAX = 2,
      PAYLOAD_LEN = 4
    };

  public:
    DataMaxAcceleration(void *input, size_t msg_len);

    DataMaxAcceleration(const DataMaxAcceleration &other);

    static DataMaxAcceleration *popNext();

    static DataMaxAcceleration *waitNext(double timeout = 0);

    static DataMaxAcceleration *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getForwardMax();

    double getReverseMax();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataMaxSpeed : public Message
  {
  public:
    enum payloadOffsets
    {
      FORWARD_MAX = 0,
      REVERSE_MAX = 2,
      PAYLOAD_LEN = 4
    };

  public:
    DataMaxSpeed(void *input, size_t msg_len);

    DataMaxSpeed(const DataMaxSpeed &other);

    static DataMaxSpeed *popNext();

    static DataMaxSpeed *waitNext(double timeout = 0);

    static DataMaxSpeed *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getForwardMax();

    double getReverseMax();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformAcceleration : public Message
  {
  public:
    enum payloadOffsets
    {
      X = 0,
      Y = 2,
      Z = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataPlatformAcceleration(void *input, size_t msg_len);

    DataPlatformAcceleration(const DataPlatformAcceleration &other);

    static DataPlatformAcceleration *popNext();

    static DataPlatformAcceleration *waitNext(double timeout = 0);

    static DataPlatformAcceleration *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq = 0);

    static enum MessageTypes getTypeID();

    double getX();

    double getY();

    double getZ();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformInfo : public Message
  {
  private:
    uint8_t strlenModel();

  public:
    DataPlatformInfo(void *input, size_t msg_len);

    DataPlatformInfo(const DataPlatformInfo &other);

    static DataPlatformInfo *popNext();

    static DataPlatformInfo *waitNext(double timeout = 0);

    static DataPlatformInfo *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    std::string getModel();

    uint8_t getRevision();

    uint32_t getSerial();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformName : public Message
  {
  public:
    DataPlatformName(void *input, size_t msg_len);

    DataPlatformName(const DataPlatformName &other);

    static DataPlatformName *popNext();

    static DataPlatformName *waitNext(double timeout = 0);

    static DataPlatformName *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    std::string getName();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformMagnetometer : public Message
  {
  public:
    enum payloadOffsets
    {
      X = 0,
      Y = 2,
      Z = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataPlatformMagnetometer(void *input, size_t msg_len);

    DataPlatformMagnetometer(const DataPlatformMagnetometer &other);

    static DataPlatformMagnetometer *popNext();

    static DataPlatformMagnetometer *waitNext(double timeout = 0);

    static DataPlatformMagnetometer *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getX();

    double getY();

    double getZ();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformOrientation : public Message
  {
  public:
    enum payloadOffsets
    {
      ROLL = 0,
      PITCH = 2,
      YAW = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataPlatformOrientation(void *input, size_t msg_len);

    DataPlatformOrientation(const DataPlatformOrientation &other);

    static DataPlatformOrientation *popNext();

    static DataPlatformOrientation *waitNext(double timeout = 0);

    static DataPlatformOrientation *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getRoll();

    double getYaw();

    double getPitch();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPlatformRotation : public Message
  {
  public:
    enum payloadOffsets
    {
      ROLL_RATE = 0,
      PITCH_RATE = 2,
      YAW_RATE = 4,
      PAYLOAD_LEN = 6
    };

  public:
    DataPlatformRotation(void *input, size_t msg_len);

    DataPlatformRotation(const DataPlatformRotation &other);

    static DataPlatformRotation *popNext();

    static DataPlatformRotation *waitNext(double timeout = 0);

    static DataPlatformRotation *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getRollRate();

    double getPitchRate();

    double getYawRate();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataPowerSystem : public Message
  {
  public:
    class BatteryDescription
    {
    public:
      enum Types
      {
        EXTERNAL = 0x0,
        LEAD_ACID = 0x1,
        NIMH = 0x2,
        GASOLINE = 0x8
      };
      uint8_t rawDesc;
    public:
      BatteryDescription(uint8_t desc) : rawDesc(desc)
      {
      }

      bool isPresent()
      {
        return rawDesc & 0x80;
      }

      bool isInUse()
      {
        return rawDesc & 0x40;
      }

      enum Types getType()
      {
        return (enum Types) (rawDesc & 0x0f);
      }
    };

  public:
    DataPowerSystem(void *input, size_t msg_len);

    DataPowerSystem(const DataPowerSystem &other);

    static DataPowerSystem *popNext();

    static DataPowerSystem *waitNext(double timeout = 0);

    static DataPowerSystem *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getBatteryCount();

    double getChargeEstimate(uint8_t battery);

    int16_t getCapacityEstimate(uint8_t battery);

    BatteryDescription getDescription(uint8_t battery);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataProcessorStatus : public Message
  {
  public:
    DataProcessorStatus(void *input, size_t msg_len);

    DataProcessorStatus(const DataProcessorStatus &other);

    static DataProcessorStatus *popNext();

    static DataProcessorStatus *waitNext(double timeout = 0);

    static DataProcessorStatus *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getProcessCount();

    int16_t getErrorCount(int process);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRangefinders : public Message
  {
  public:
    DataRangefinders(void *input, size_t msg_len);

    DataRangefinders(const DataRangefinders &other);

    static DataRangefinders *popNext();

    static DataRangefinders *waitNext(double timeout = 0);

    static DataRangefinders *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getRangefinderCount();

    int16_t getDistance(int rangefinder);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRangefinderTimings : public Message
  {
  public:
    DataRangefinderTimings(void *input, size_t msg_len);

    DataRangefinderTimings(const DataRangefinderTimings &other);

    static DataRangefinderTimings *popNext();

    static DataRangefinderTimings *waitNext(double timeout = 0);

    static DataRangefinderTimings *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getRangefinderCount();

    int16_t getDistance(int rangefinder);

    uint32_t getAcquisitionTime(int rangefinder);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawAcceleration : public Message
  {
  public:
    enum payloadOffsets
    {
      X = 0,
      Y = 2,
      Z = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataRawAcceleration(void *input, size_t msg_len);

    DataRawAcceleration(const DataRawAcceleration &other);

    static DataRawAcceleration *popNext();

    static DataRawAcceleration *waitNext(double timeout = 0);

    static DataRawAcceleration *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint16_t getX();

    uint16_t getY();

    uint16_t getZ();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawCurrent : public Message
  {
  public:
    DataRawCurrent(void *input, size_t msg_len);

    DataRawCurrent(const DataRawCurrent &other);

    static DataRawCurrent *popNext();

    static DataRawCurrent *waitNext(double timeout = 0);

    static DataRawCurrent *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getCurrentCount();

    uint16_t getCurrent(int current);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawGyro : public Message
  {
  public:
    enum payloadOffsets
    {
      ROLL = 0,
      PITCH = 2,
      YAW = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataRawGyro(void *input, size_t msg_len);

    DataRawGyro(const DataRawGyro &other);

    static DataRawGyro *popNext();

    static DataRawGyro *waitNext(double timeout = 0);

    static DataRawGyro *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint16_t getRoll();

    uint16_t getPitch();

    uint16_t getYaw();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawMagnetometer : public Message
  {
  public:
    enum payloadOffsets
    {
      X = 0,
      Y = 2,
      Z = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataRawMagnetometer(void *input, size_t msg_len);

    DataRawMagnetometer(const DataRawMagnetometer &other);

    static DataRawMagnetometer *popNext();

    static DataRawMagnetometer *waitNext(double timeout = 0);

    static DataRawMagnetometer *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint16_t getX();

    uint16_t getY();

    uint16_t getZ();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawOrientation : public Message
  {
  public:
    enum payloadOffsets
    {
      ROLL = 0,
      PITCH = 2,
      YAW = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataRawOrientation(void *input, size_t msg_len);

    DataRawOrientation(const DataRawOrientation &other);

    static DataRawOrientation *popNext();

    static DataRawOrientation *waitNext(double timeout = 0);

    static DataRawOrientation *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint16_t getRoll();

    uint16_t getPitch();

    uint16_t getYaw();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawTemperature : public Message
  {
  public:
    DataRawTemperature(void *input, size_t msg_len);

    DataRawTemperature(const DataRawTemperature &other);

    static DataRawTemperature *popNext();

    static DataRawTemperature *waitNext(double timeout = 0);

    static DataRawTemperature *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getTemperatureCount();

    uint16_t getTemperature(int temperature);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataRawVoltage : public Message
  {
  public:
    DataRawVoltage(void *input, size_t msg_len);

    DataRawVoltage(const DataRawVoltage &other);

    static DataRawVoltage *popNext();

    static DataRawVoltage *waitNext(double timeout = 0);

    static DataRawVoltage *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint8_t getVoltageCount();

    uint16_t getVoltage(int temperature);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataSafetySystemStatus : public Message
  {
  public:
    DataSafetySystemStatus(void *input, size_t msg_len);

    DataSafetySystemStatus(const DataSafetySystemStatus &other);

    static DataSafetySystemStatus *popNext();

    static DataSafetySystemStatus *waitNext(double timeout = 0);

    static DataSafetySystemStatus *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint16_t getFlags();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataSystemStatus : public Message
  {
  private:
    uint8_t voltages_offset;
    uint8_t currents_offset;
    uint8_t temperatures_offset;

  public:
    DataSystemStatus(void *input, size_t msg_len);

    DataSystemStatus(const DataSystemStatus &other);

    static DataSystemStatus *popNext();

    static DataSystemStatus *waitNext(double timeout = 0);

    static DataSystemStatus *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    uint32_t getUptime();

    uint8_t getVoltagesCount();

    double getVoltage(uint8_t index);

    uint8_t getCurrentsCount();

    double getCurrent(uint8_t index);

    uint8_t getTemperaturesCount();

    double getTemperature(uint8_t index);

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

  class DataVelocity : public Message
  {
  public:
    enum payloadOffsets
    {
      TRANS_VEL = 0,
      ROTATIONAL = 2,
      TRANS_ACCEL = 4,
      PAYLOAD_LEN = 6
    };
  public:
    DataVelocity(void *input, size_t msg_len);

    DataVelocity(const DataVelocity &other);

    static DataVelocity *popNext();

    static DataVelocity *waitNext(double timeout = 0);

    static DataVelocity *getUpdate(double timeout = 0);

    static void subscribe(uint16_t freq);

    static enum MessageTypes getTypeID();

    double getTranslational();

    double getRotational();

    double getTransAccel();

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);
  };

} // namespace clearpath

#endif  // CLEARPATH_MESSAGE_DATA_H
