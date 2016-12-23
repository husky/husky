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
*  File: Message.cpp
*  Desc: Definitions of the Message class. This class represents a
*        single message which is sent or received from a platform
*  Auth: R. Gariepy
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

#include <unistd.h>
#include <iostream>
#include <string.h>
#include "husky_base/horizon_legacy/crc.h"
#include "husky_base/horizon_legacy/Message.h"
#include "husky_base/horizon_legacy/Message_data.h"
#include "husky_base/horizon_legacy/Number.h"
#include "husky_base/horizon_legacy/Transport.h"

// Conditions on the below to handle compiling for nonstandard hardware
#ifdef LOGGING_AVAIL
#include "husky_base/horizon_legacy/Logger.h"
#endif

using namespace std;

namespace clearpath
{

  MessageException::MessageException(const char *msg, enum errors ex_type)
      : Exception(msg), type(ex_type)
  {
#ifdef LOGGING_AVAIL
	CPR_EXCEPT() << "MessageException "<<type<<": "<< message << endl << flush;
#endif
  }

  Message::Message() :
      is_sent(false)
  {
    total_len = HEADER_LENGTH + CRC_LENGTH;
    memset(data, 0, MAX_MSG_LENGTH);
  }

  Message::Message(void *input, size_t msg_len) :
      is_sent(false)
  {
    total_len = msg_len;
    memset(data, 0, MAX_MSG_LENGTH);
    memcpy(data, input, msg_len);
  }

  Message::Message(const Message &other) :
      is_sent(false)
  {
    total_len = other.total_len;
    memset(data, 0, MAX_MSG_LENGTH);
    memcpy(data, other.data, total_len);
  }

  Message::Message(uint16_t type, uint8_t *payload, size_t payload_len,
      uint32_t timestamp, uint8_t flags, uint8_t version) :
      is_sent(false)
  {
    /* Copy in data */
    total_len = HEADER_LENGTH + payload_len + CRC_LENGTH;
    if (total_len > MAX_MSG_LENGTH)
    {
      /* If payload is too long, the only recourse we have in constructor
       * (other than an abort()) is to truncate silently. */
      total_len = MAX_MSG_LENGTH;
      payload_len = MAX_MSG_LENGTH - HEADER_LENGTH - CRC_LENGTH;
    }
    memset(data, 0, MAX_MSG_LENGTH);
    memcpy(data + PAYLOAD_OFST, payload, payload_len);

    /* Fill header */
    data[SOH_OFST] = SOH;
    setLength(total_len - 3);
    setType(type);
    setTimestamp(timestamp);
    setFlags(flags);
    setVersion(version);
    data[STX_OFST] = STX;

    /* Generate checksum */
    uint16_t checksum = crc16(crcOffset(), CRC_INIT_VAL, data);
    utob(data + crcOffset(), 2, checksum);
  }

  Message::~Message()
  {
    // nothing to do, actually.
  }

  void Message::send()
  {
    // We will retry up to 3 times if we receive CRC errors
    for (int i = 0; i < 2; ++i)
    {
      try
      {
        Transport::instance().send(this);
        return;
      }
      catch (BadAckException *ex)
      {
        // Any bad ack other than bad checksum needs to
        // be thrown on
        if (ex->ack_flag != BadAckException::BAD_CHECKSUM)
        {
          throw ex;
        }
      }
    }

    /* Make the final attempt outside the try, so any exception
     * just goes straight through */
#ifdef LOGGING_AVAIL
    CPR_WARN() << "Bad checksum twice in a row." << endl;
#endif
    Transport::instance().send(this);
  }

/**
* Copies message payload into a provided buffer.
* @param buf       The buffer to fill
* @param buf_size  Maximum length of buf
* @return number of bytes copied.
*/
  size_t Message::getPayload(void *buf, size_t buf_size)
  {
    // If we don't have enough space in the buffer, don't even try
    if (getPayloadLength() > buf_size) { return 0; }

    memcpy(buf, data + PAYLOAD_OFST, getPayloadLength());
    return getPayloadLength();
  }

/**
* Get a pointer to the payload withing this Message's internal storage.
* @param offset    The offset from the beginning of the payload.
* @return a pointer to this Message's internal storage.
*/
  uint8_t *Message::getPayloadPointer(size_t offset)
  {
    return data + PAYLOAD_OFST + offset;
  }

  uint8_t  Message::getLength()
  {
    return data[LENGTH_OFST];
  }

  uint8_t  Message::getLengthComp()
  {
    return data[LENGTH_COMP_OFST];
  }

  uint8_t  Message::getVersion()
  {
    return data[VERSION_OFST];
  }

  uint32_t Message::getTimestamp()
  {
    return btou(data + TIMESTAMP_OFST, 4);
  }

  uint8_t  Message::getFlags()
  {
    return data[FLAGS_OFST];
  }

  uint16_t Message::getType()
  {
    return btou(data + TYPE_OFST, 2);
  }

  uint16_t Message::getChecksum()
  {
    return btou(data + crcOffset(), 2);
  }

  void Message::setLength(uint8_t len)
  {
    size_t new_total_len = len + 3;
    if (new_total_len > MAX_MSG_LENGTH) { return; }
    total_len = new_total_len;
    data[LENGTH_OFST] = len;
    data[LENGTH_COMP_OFST] = ~len;
  }

  void Message::setVersion(uint8_t version)
  {
    data[VERSION_OFST] = version;
  }

  void Message::setTimestamp(uint32_t timestamp)
  {
    utob(data + TIMESTAMP_OFST, 4, timestamp);
  }

  void Message::setFlags(uint8_t flags)
  {
    data[FLAGS_OFST] = flags;
  }

  void Message::setType(uint16_t type)
  {
    utob(data + TYPE_OFST, 2, type);
  }

/**
* Changes the payload length of the packet.
* Does not update packet len/~len fields or the checksum.  Call
* makeValid() to update these fields.
* @param len   The new payload length
*/
  void Message::setPayloadLength(uint8_t len)
  {

    if (((size_t) (len) + HEADER_LENGTH + CRC_LENGTH) > MAX_MSG_LENGTH) { return; }
    total_len = len + HEADER_LENGTH + CRC_LENGTH;
  }

/**
* Set the payload of this message.  Modifies the length of the
* message as necessary, as per setPayloadLength.
* @see Message::setPayloadLength()
* @param buf       Buffer containing the new payload.
* @param buf_size  Length of buf.
*/
  void Message::setPayload(void *buf, size_t buf_size)
  {
    if ((buf_size + HEADER_LENGTH + CRC_LENGTH) > MAX_MSG_LENGTH) { return; }
    setPayloadLength(buf_size);
    if (buf_size > getPayloadLength()) { return; }
    memcpy(data + PAYLOAD_OFST, buf, buf_size);
  }

/**
* Copy the complete raw content of this message to a buffer.
* @param buf       The buffer to copy to
* @param buf_size  The maximum length of buf
* @return buf on success, NULL on failure
*/
  size_t Message::toBytes(void *buf, size_t buf_size)
  {
    // If we don't have enough space in the buffer, don't even try
    if (total_len > buf_size)
    {
      return 0;
    }
    memcpy(buf, data, total_len);
    return total_len;
  }

/**
* Checks the consistency of this message.
* @param whyNot    Optionally, a reason for validation failure will be
*                  written here.
* @param strLen    Length of the optional whyNot string
* @return true if the message is valid, false otherwise.
*/
  bool Message::isValid(char *whyNot, size_t strLen)
  {
    // Check SOH 
    if (data[SOH_OFST] != SOH)
    {
      if (whyNot) { strncpy(whyNot, "SOH is not present.", strLen); }
      return false;
    }
    // Check STX
    if (data[STX_OFST] != STX)
    {
      if (whyNot) { strncpy(whyNot, "STX is not present.", strLen); }
      return false;
    }
    // Check length matches complement
    if (getLength() != ((~getLengthComp()) & 0xff))
    {
      if (whyNot) { strncpy(whyNot, "Length does not match complement.", strLen); }
      return false;
    }
    // Check length is correct
    if (getLength() != (total_len - 3))
    {
      if (whyNot) { strncpy(whyNot, "Length is wrong.", strLen); }
      return false;
    }
    // Check the CRC
    if (crc16(crcOffset(), CRC_INIT_VAL, this->data) != getChecksum())
    {
      if (whyNot) { strncpy(whyNot, "CRC is wrong.", strLen); }
      return false;
    }
    return true;
  }

/**
* Sets SOH, STX, length, and checksum so that this message becomes valid.
*/
  void Message::makeValid()
  {
    data[SOH_OFST] = SOH;
    data[STX_OFST] = STX;
    data[LENGTH_OFST] = total_len - 3;
    data[LENGTH_COMP_OFST] = ~data[LENGTH_OFST];
    uint16_t checksum = crc16(crcOffset(), CRC_INIT_VAL, data);
    utob(data + crcOffset(), 2, checksum);
  }

  std::ostream &Message::printMessage(std::ostream &stream)
  {
    stream << "Message" << endl;
    stream << "=======" << endl;
    stream << "Length   : " << (int) (getLength()) << endl;
    stream << "~Length  : " << (int) (getLengthComp()) << endl;
    stream << "Version  : " << (int) (getVersion()) << endl;
    stream << "Flags    : " << hex << (int) (getFlags()) << endl;
    stream << "Timestamp: " << dec << getTimestamp() << endl;
    stream << "Type     : " << hex << (int) (getType()) << endl;
    stream << "Checksum : " << hex << (int) (getChecksum()) << endl;
    stream << dec;
    stream << "Raw      : ";
    printRaw(stream);
    return stream;
  }

  void Message::printRaw(std::ostream &stream)
  {
    stream << hex << uppercase;
    for (unsigned int i = 0; i < total_len; i++)
    {
      stream << static_cast<short>(data[i]) << " ";
    }
    stream << dec;
    stream << endl;
  }

/**
* Instantiates the Message subclass corresponding to the
* type field in raw message data.
* @param input     The raw message data to instantiate from
* @param msg_len   The length of input.
* @return  An instance of the correct Message subclass
*/
  Message *Message::factory(void *input, size_t msg_len)
  {
    uint16_t type = btou((char *) input + TYPE_OFST, 2);

    switch (type)
    {
      case DATA_ACCEL:
        return new DataPlatformAcceleration(input, msg_len);

      case DATA_ACCEL_RAW:
        return new DataRawAcceleration(input, msg_len);

      case DATA_ACKERMANN_SETPTS:
        return new DataAckermannOutput(input, msg_len);

      case DATA_CURRENT_RAW:
        return new DataRawCurrent(input, msg_len);

      case DATA_PLATFORM_NAME:
        return new DataPlatformName(input, msg_len);

      case DATA_DIFF_CTRL_CONSTS:
        return new DataDifferentialControl(input, msg_len);

      case DATA_DIFF_WHEEL_SPEEDS:
        return new DataDifferentialSpeed(input, msg_len);

      case DATA_DIFF_WHEEL_SETPTS:
        return new DataDifferentialOutput(input, msg_len);

      case DATA_DISTANCE_DATA:
        return new DataRangefinders(input, msg_len);

      case DATA_DISTANCE_TIMING:
        return new DataRangefinderTimings(input, msg_len);

      case DATA_ECHO:
        return new DataEcho(input, msg_len);

      case DATA_ENCODER:
        return new DataEncoders(input, msg_len);

      case DATA_ENCODER_RAW:
        return new DataEncodersRaw(input, msg_len);

      case DATA_FIRMWARE_INFO:
        return new DataFirmwareInfo(input, msg_len);

      case DATA_GYRO_RAW:
        return new DataRawGyro(input, msg_len);

      case DATA_MAGNETOMETER:
        return new DataPlatformMagnetometer(input, msg_len);

      case DATA_MAGNETOMETER_RAW:
        return new DataRawMagnetometer(input, msg_len);

      case DATA_MAX_ACCEL:
        return new DataMaxAcceleration(input, msg_len);

      case DATA_MAX_SPEED:
        return new DataMaxSpeed(input, msg_len);

      case DATA_ORIENT:
        return new DataPlatformOrientation(input, msg_len);

      case DATA_ORIENT_RAW:
        return new DataRawOrientation(input, msg_len);

      case DATA_PLATFORM_INFO:
        return new DataPlatformInfo(input, msg_len);

      case DATA_POWER_SYSTEM:
        return new DataPowerSystem(input, msg_len);

      case DATA_PROC_STATUS:
        return new DataProcessorStatus(input, msg_len);

      case DATA_ROT_RATE:
        return new DataPlatformRotation(input, msg_len);

      case DATA_SAFETY_SYSTEM:
        return new DataSafetySystemStatus(input, msg_len);

      case DATA_SYSTEM_STATUS:
        return new DataSystemStatus(input, msg_len);

      case DATA_TEMPERATURE_RAW:
        return new DataRawTemperature(input, msg_len);

      case DATA_VELOCITY_SETPT:
        return new DataVelocity(input, msg_len);

      case DATA_VOLTAGE_RAW:
        return new DataRawVoltage(input, msg_len);

      default:
        return new Message(input, msg_len);
    } // switch getType()
  } // factory()

  Message *Message::popNext()
  {
    return Transport::instance().popNext();
  }

  Message *Message::waitNext(double timeout)
  {
    return Transport::instance().waitNext(timeout);
  }

}; // namespace clearpath

std::ostream &operator<<(std::ostream &stream, clearpath::Message &msg)
{
  return msg.printMessage(stream);
}

