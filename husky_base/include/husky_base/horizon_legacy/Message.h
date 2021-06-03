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
*             ROBOTICS�
*
*  File: Message.h
*  Desc: Definition for the Message class. This class represents a
*        single message which is sent or received from a platform
*  Auth: R. Gariepy, Iain Peet
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

#ifndef CLEARPATH_MESSAGE_H
#define CLEARPATH_MESSAGE_H

#include <iostream>
#include <cstdlib>
#include <stdint.h>

#include "husky_base/horizon_legacy/Exception.h"


namespace clearpath
{

  class MessageException : public Exception
  {
  public:
    enum errors
    {
      ERROR_BASE = 0,
      INVALID_LENGTH
    };

  public:
    enum errors type;

    MessageException(const char *msg, enum errors ex_type = ERROR_BASE);
  };

  class Message
  {
  public:
    static const size_t MAX_MSG_LENGTH = 256;


  protected:
    static const size_t CRC_LENGTH = 2;
    static const uint16_t CRC_INIT_VAL = 0xFFFF;

    static const size_t HEADER_LENGTH = 12;

    // Offsets of fields within data
    enum dataOffsets
    {
      SOH_OFST = 0,
      LENGTH_OFST,
      LENGTH_COMP_OFST,
      VERSION_OFST,
      TIMESTAMP_OFST,
      FLAGS_OFST = 8,
      TYPE_OFST,
      STX_OFST = 11,
      PAYLOAD_OFST
    };

    uint8_t data[MAX_MSG_LENGTH];
    // Total length (incl. full header & checksum)
    size_t total_len;

    // Whether this Message has ever been sent by the Transport()
    // (Updated by Transport::send())
    bool is_sent;

    friend class Transport;  // Allow Transport to read data and total_len directly

  public:
    static const size_t MIN_MSG_LENGTH = HEADER_LENGTH + CRC_LENGTH;
    static const uint8_t SOH = 0xAA;
    static const uint8_t STX = 0x55;

  protected:
    size_t crcOffset()
    {
      return total_len - CRC_LENGTH;
    };

    void setLength(uint8_t len);

    void setVersion(uint8_t version);

    void setTimestamp(uint32_t timestamp);

    void setFlags(uint8_t flags);

    void setType(uint16_t type);

    uint8_t *getPayloadPointer(size_t offset = 0);

    void setPayload(void *buf, size_t buf_size);

    void setPayloadLength(uint8_t len);

    void makeValid();

  public:
    Message();

    Message(void *input, size_t msg_len);

    Message(const Message &other);

    Message(uint16_t type, uint8_t *payload, size_t payload_len,
        uint32_t timestamp = 0, uint8_t flags = 0, uint8_t version = 0);

    virtual ~Message();

    void send();

    uint8_t getLength();  // as reported by packet length field.
    uint8_t getLengthComp();

    uint8_t getVersion();

    uint32_t getTimestamp();

    uint8_t getFlags();

    uint16_t getType();

    uint16_t getChecksum();

    size_t getPayloadLength()
    {
      return total_len - HEADER_LENGTH - CRC_LENGTH;
    }

    size_t getPayload(void *buf, size_t max_size);

    size_t getTotalLength()
    {
      return total_len;
    }

    size_t toBytes(void *buf, size_t buf_size);

    bool isValid(char *whyNot = NULL, size_t strLen = 0);

    bool isCommand()
    {
      return getType() < 0x4000;
    }

    bool isRequest()
    {
      return (getType() >= 0x4000) && (getType() < 0x8000);
    }

    bool isData()
    {
      return (getType() >= 0x8000) && (getType() < 0xC000);
    }

    virtual std::ostream &printMessage(std::ostream &stream = std::cout);

    void printRaw(std::ostream &stream = std::cout);


    static Message *factory(void *input, size_t msg_len);

    static Message *popNext();

    static Message *waitNext(double timeout = 0.0);

  }; // class Message

  enum MessageTypes
  {
    /*
     * Set commands
     */
        SET_PLATFORM_NAME = 0x0002,
    SET_PLATFORM_TIME = 0x0005,
    SET_SAFETY_SYSTEM = 0x0010,
    SET_DIFF_WHEEL_SPEEDS = 0x0200,
    SET_DIFF_CTRL_CONSTS = 0x0201,
    SET_DIFF_WHEEL_SETPTS = 0x0202,
    SET_ACKERMANN_SETPT = 0x0203,
    SET_VELOCITY_SETPT = 0x0204,
    SET_TURN_SETPT = 0x0205,
    SET_MAX_SPEED = 0x0210,
    SET_MAX_ACCEL = 0x0211,
    SET_GEAR_SETPOINT = 0x0212,
    SET_GPADC_OUTPUT = 0x0300,
    SET_GPIO_DIRECTION = 0x0301,
    SET_GPIO_OUTPUT = 0x0302,
    SET_PTZ_POSITION = 0x0400,

    /*
     * Command commands
     */
        CMD_PROCESSOR_RESET = 0x2000,
    CMD_RESTORE_SETTINGS = 0x2001,
    CMD_STORE_SETTINGS = 0x2002,

    /*
     * Request commands
     */
        REQUEST_ECHO = 0x4000,
    REQUEST_PLATFORM_INFO = 0x4001,
    REQUEST_PLATFORM_NAME = 0x4002,
    REQUEST_FIRMWARE_INFO = 0x4003,
    REQUEST_SYSTEM_STATUS = 0x4004,
    REQUEST_POWER_SYSTEM = 0x4005,
    REQUEST_SAFETY_SYSTEM = 0x4010,
    REQUEST_DIFF_WHEEL_SPEEDS = 0x4200,
    REQUEST_DIFF_CTRL_CONSTS = 0x4201,
    REQUEST_DIFF_WHEEL_SETPTS = 0x4202,
    REQUEST_ACKERMANN_SETPTS = 0x4203,
    REQUEST_VELOCITY_SETPT = 0x4204,
    REQUEST_TURN_SETPT = 0x4205,
    REQUEST_MAX_SPEED = 0x4210,
    REQUEST_MAX_ACCEL = 0x4211,
    REQUEST_GEAR_SETPT = 0x4212,
    REQUEST_GPADC_OUTPUT = 0x4300,
    REQUEST_GPIO_STATUS = 0x4301,
    REQUEST_GPADC_INPUT = 0x4303,
    REQUEST_PTZ_POSITION = 0x4400,
    REQUEST_DISTANCE_DATA = 0x4500,
    REQUEST_DISTANCE_TIMING = 0x4501,
    REQUEST_ORIENT = 0x4600,
    REQUEST_ROT_RATE = 0x4601,
    REQUEST_ACCEL = 0x4602,
    REQUEST_6AXIS = 0x4603,
    REQUEST_6AXIS_ORIENT = 0x4604,
    REQUEST_ENCODER = 0x4800,
    REQUEST_ENCODER_RAW = 0x4801,

    /*
     * Data commands
     */
        DATA_ECHO = 0x8000,
    DATA_PLATFORM_INFO = 0x8001,
    DATA_PLATFORM_NAME = 0x8002,
    DATA_FIRMWARE_INFO = 0x8003,
    DATA_SYSTEM_STATUS = 0x8004,
    DATA_POWER_SYSTEM = 0x8005,
    DATA_PROC_STATUS = 0x8006,
    DATA_SAFETY_SYSTEM = 0x8010,
    DATA_DIFF_WHEEL_SPEEDS = 0x8200,
    DATA_DIFF_CTRL_CONSTS = 0x8201,
    DATA_DIFF_WHEEL_SETPTS = 0x8202,
    DATA_ACKERMANN_SETPTS = 0x8203,
    DATA_VELOCITY_SETPT = 0x8204,
    DATA_TURN_SETPT = 0x8205,
    DATA_MAX_SPEED = 0x8210,
    DATA_MAX_ACCEL = 0x8211,
    DATA_GEAR_SETPT = 0x8212,
    DATA_GPADC_OUTPUT = 0x8300,
    DATA_GPIO_STATUS = 0x8301,
    DATA_GPADC_INPUT = 0x8303,
    DATA_PTZ_POSITION = 0x8400,
    DATA_DISTANCE_DATA = 0x8500,
    DATA_DISTANCE_TIMING = 0x8501,
    DATA_ORIENT = 0x8600,
    DATA_ROT_RATE = 0x8601,
    DATA_ACCEL = 0x8602,
    DATA_6AXIS = 0x8603,
    DATA_6AXIS_ORIENT = 0x8604,
    DATA_MAGNETOMETER = 0x8606,
    DATA_ENCODER = 0x8800,
    DATA_ENCODER_RAW = 0x8801,
    DATA_CURRENT_RAW = 0xA110,
    DATA_VOLTAGE_RAW = 0xA111,
    DATA_TEMPERATURE_RAW = 0xA112,
    DATA_ORIENT_RAW = 0xA113,
    DATA_GYRO_RAW = 0xA114,
    DATA_ACCEL_RAW = 0xA115,
    DATA_MAGNETOMETER_RAW = 0xA116
  }; // enum MessageTypes

} // namespace clearpath

std::ostream &operator<<(std::ostream &stream, clearpath::Message &msg);

#endif  // CLEARPATH_MESSAGE_H
