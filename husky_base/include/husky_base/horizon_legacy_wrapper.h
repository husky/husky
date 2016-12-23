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

#ifndef HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
#define HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H

#include "husky_base/horizon_legacy/clearpath.h"
#include "boost/type_traits/is_base_of.hpp"

namespace
{
  const uint16_t UNSUBSCRIBE = 0xFFFF;
}

namespace horizon_legacy
{

  void connect(std::string port);

  void reconnect();

  void configureLimits(double max_speed, double max_accel);

  void controlSpeed(double speed_left, double speed_right, double accel_left, double accel_right);

  template<typename T>
  struct Channel
  {

    typedef boost::shared_ptr<T> Ptr;
    typedef boost::shared_ptr<const T> ConstPtr;
    BOOST_STATIC_ASSERT_MSG(
      (boost::is_base_of<clearpath::Message, T>::value),
      "T must be a descendant of clearpath::Message"
    );

    static Ptr getLatest(double timeout)
    {
      T *latest = 0;

      // Iterate over all messages in queue and find the latest
      while (T *next = T::popNext())
      {
        if (latest)
        {
          delete latest;
          latest = 0;
        }
        latest = next;
      }

      // If no messages found in queue, then poll for timeout until one is received
      if (!latest)
      {
        latest = T::waitNext(timeout);
      }

      // If no messages received within timeout, make a request
      if (!latest)
      {
        return requestData(timeout);
      }

      return Ptr(latest);
    }

    static Ptr requestData(double timeout)
    {
      T *update = 0;
      while (!update)
      {
        update = T::getUpdate(timeout);
        if (!update)
        {
          reconnect();
        }
      }
      return Ptr(update);
    }

    static void subscribe(double frequency)
    {
      T::subscribe(frequency);
    }

    static void unsubscribe()
    {
      T::subscribe(UNSUBSCRIBE);
    }

  };

} // namespace husky_base
#endif  // HUSKY_BASE_HORIZON_LEGACY_WRAPPER_H
