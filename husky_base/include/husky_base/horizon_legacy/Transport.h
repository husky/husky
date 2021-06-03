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
*  File: Transport.h
*  Desc: Definition for Horizon transport class. Implements the details of
*        the Horizon communication medium, providing the ability to send
*        and receive messages.  Received messages are queued.
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

#ifndef CLEARPATH_TRANSPORT_H
#define CLEARPATH_TRANSPORT_H

#include <list>
#include <iostream>

#include "husky_base/horizon_legacy/Message.h"
#include "husky_base/horizon_legacy/Exception.h"

namespace clearpath
{

  class TransportException : public Exception
  {
  public:
    enum errors
    {
      ERROR_BASE,
      NOT_CONFIGURED,
      CONFIGURE_FAIL,
      UNACKNOWLEDGED_SEND,
      BAD_ACK_RESULT
    };
  public:
    enum errors type;

    TransportException(const char *msg, enum errors ex_type = ERROR_BASE);
  };

  class BadAckException : public TransportException
  {
  public:
    enum ackFlags
    {
      BAD_CHECKSUM = 0x01,
      BAD_TYPE = 0x02,
      BAD_FORMAT = 0x04,
      RANGE = 0x08,
      NO_BANDWIDTH = 0x10,
      OVER_FREQ = 0x20,
      OVER_SUBSCRIBE = 0x40
    } ack_flag;

    BadAckException(unsigned int flag);
  };

/*
 * Transport class
 */
  class Transport
  {
  public:
    enum counterTypes
    {
      GARBLE_BYTES, // bytes with no SOH / bad length
      INVALID_MSG,  // bad format / CRC wrong
      IGNORED_ACK,  // ack we didn't care about
      QUEUE_FULL,   // dropped msg because of overfull queue
      NUM_COUNTERS  // end of list, not actual counter
    };
    static const char *counter_names[NUM_COUNTERS]; // N.B: must be updated with counterTypes


  private:
    bool configured;
    void *serial;
    int retries;

    static const int RETRY_DELAY_MS = 200;

    std::list<Message *> rx_queue;
    static const size_t MAX_QUEUE_LEN = 10000;

    unsigned long counters[NUM_COUNTERS];

  private:
    Message *rxMessage();

    Message *getAck();

    void enqueueMessage(Message *msg);

    int openComm(const char *device);

    int closeComm();

    void resetCounters();

  protected:
    Transport();

    ~Transport();

  public:
    static Transport &instance();

    void configure(const char *device, int retries);

    bool isConfigured()
    {
      return configured;
    }

    int close();

    void poll();

    void send(Message *m);

    Message *popNext();

    Message *popNext(enum MessageTypes type);

    Message *waitNext(double timeout = 0.0);

    Message *waitNext(enum MessageTypes type, double timeout = 0.0);

    void flush(std::list<Message *> *queue = 0);

    void flush(enum MessageTypes type, std::list<Message *> *queue = 0);

    unsigned long getCounter(enum counterTypes counter)
    {
      return counters[counter];
    }

    void printCounters(std::ostream &stream = std::cout);
  };

} // namespace clearpath

#endif // CLEARPATH_TRANSPORT_H
