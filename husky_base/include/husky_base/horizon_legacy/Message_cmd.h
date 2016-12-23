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
*  File: Message_cmd.h
*  Desc: Provides Set Message subclasses.
*  Auth: Iain Peet
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

#ifndef CLEARPATH_MESSAGE_CMD_H
#define CLEARPATH_MESSAGE_CMD_H

#include "husky_base/horizon_legacy/Message.h"

namespace clearpath
{

  class CmdMessage : public Message
  {
  private:
    static long total_destroyed;
    static long total_sent;

  public:
    CmdMessage() : Message()
    {
    }

    CmdMessage(const CmdMessage &other) : Message(other)
    {
    }

    virtual ~CmdMessage();
  };

  class CmdProcessorReset : public CmdMessage
  {
  public:
    CmdProcessorReset();

    CmdProcessorReset(const CmdProcessorReset &other);
  };

  class CmdRestoreSettings : public CmdMessage
  {
  public:
    enum restoreFlags
    {
      USER_SETTINGS = 0x1,
      FACTORY_SETTINGS = 0x2
    };
  public:
    CmdRestoreSettings(enum restoreFlags flags);

    CmdRestoreSettings(const CmdRestoreSettings &other);
  };

  class CmdStoreSettings : public CmdMessage
  {
  public:
    CmdStoreSettings();

    CmdStoreSettings(const CmdStoreSettings &other);
  };

  class SetAckermannOutput : public CmdMessage
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
    SetAckermannOutput(double steering, double throt, double brake);

    SetAckermannOutput(const SetAckermannOutput &other);
  };

  class SetDifferentialControl : public CmdMessage
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
    SetDifferentialControl(double p,
        double i,
        double d,
        double feedfwd,
        double stic,
        double int_lim);

    SetDifferentialControl(double left_p,
        double left_i,
        double left_d,
        double left_feedfwd,
        double left_stic,
        double left_int_lim,
        double right_p,
        double right_i,
        double right_d,
        double right_feedfwd,
        double right_stic,
        double right_int_lim);

    SetDifferentialControl(const SetDifferentialControl &other);
  };

  class SetDifferentialOutput : public CmdMessage
  {
  public:
    enum payloadOffsets
    {
      LEFT = 0,
      RIGHT = 2,
      PAYLOAD_LEN = 4
    };

  public:
    SetDifferentialOutput(double left, double right);

    SetDifferentialOutput(const SetDifferentialOutput &other);
  };

  class SetDifferentialSpeed : public CmdMessage
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
    SetDifferentialSpeed(double left_spd, double right_speed, double left_accel, double right_accel);

    SetDifferentialSpeed(const SetDifferentialSpeed &other);
  };

  class SetGear : public CmdMessage
  {
  public:
    SetGear(uint8_t gear);

    SetGear(const SetGear &other);
  };

  class SetMaxAccel : public CmdMessage
  {
  public:
    enum payloadOffsets
    {
      MAX_FWD = 0,
      MAX_REV = 2,
      PAYLOAD_LEN = 4
    };

  public:
    SetMaxAccel(double max_fwd, double max_rev);

    SetMaxAccel(const SetMaxAccel &other);
  };

  class SetMaxSpeed : public CmdMessage
  {
  public:
    enum payloadOffsets
    {
      MAX_FWD = 0,
      MAX_REV = 2,
      PAYLOAD_LEN = 4
    };

  public:
    SetMaxSpeed(double max_fwd, double max_rev);

    SetMaxSpeed(const SetMaxSpeed &other);
  };

  class SetPlatformName : public CmdMessage
  {
  public:
    SetPlatformName(const char *name);

    SetPlatformName(const SetPlatformName &other);
  };

  class SetPlatformTime : public CmdMessage
  {
  public:
    SetPlatformTime(uint32_t time);

    SetPlatformTime(const SetPlatformTime &other);
  };

  class SetSafetySystem : public CmdMessage
  {
  public:
    SetSafetySystem(uint16_t flags);

    SetSafetySystem(const SetSafetySystem &other);
  };

  class SetTurn : public CmdMessage
  {
  public:
    enum payloadOffsets
    {
      TRANSLATIONAL = 0,
      TURN_RADIUS = 2,
      TRANS_ACCEL = 4,
      PAYLOAD_LEN = 6
    };

  public:
    SetTurn(double trans, double rad, double accel);

    SetTurn(const SetTurn &other);
  };

  class SetVelocity : public CmdMessage
  {
  public:
    enum payloadOffsets
    {
      TRANSLATIONAL = 0,
      ROTATIONAL = 2,
      TRANS_ACCEL = 4,
      PAYLOAD_LEN = 6
    };

  public:
    SetVelocity(double trans, double rot, double accel);

    SetVelocity(const SetVelocity &other);
  };

}; // namespace clearpath

#endif // CLEARPATH_MESSAGE_CMD_H

