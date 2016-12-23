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
*  File: Number.cpp
*  Desc: Provides a family of functions similar in form to stdlib atoi and
*        friends, for conversion between numeric primitives and small-endian
*        byte arrays.
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

#include "husky_base/horizon_legacy/Number.h"
#include <cstdlib>

using namespace std;

namespace clearpath
{

  void utob(void *dest, size_t dest_len, uint64_t src)
  {
    size_t i;
    /* Copy bytes from int to array */
    for (i = 0; (i < dest_len) && (i < sizeof(uint64_t)); ++i)
    {
      ((uint8_t *) dest)[i] = (src >> (i * 8)) & 0xff;
    }
    /* If array is larger than int, 0-fill the remainder */
    for (; i < dest_len; ++i)
    {
      ((uint8_t *) dest)[i] = 0;
    }
  }

  void itob(void *dest, size_t dest_len, int64_t src)
  {
    size_t i;
    /* Copy bytes from int to array */
    for (i = 0; (i < dest_len) && (i < sizeof(int64_t)); ++i)
    {
      ((uint8_t *) dest)[i] = (src >> (i * 8)) & 0xff;
    }
    /* If array is larger than int, sign-fill the remainder */
    for (; i < dest_len; ++i)
    {
      if (((uint8_t *) dest)[dest_len - 1] & 0x80)
      { // MSB is set, int is negative
        ((uint8_t *) dest)[i] = 0xff;
      }
      else
      { // int is positive
        ((uint8_t *) dest)[i] = 0;
      }
    }
  }

  void ftob(void *dest, size_t dest_len, double src, double scale)
  {
    int64_t int_src = (src * scale);
    itob(dest, dest_len, int_src);
  }

/* Need to provide all these overloaded functions because integer promotion
 * of smaller int types is ambiguous between the uint64/int64 */
  void utob(void *dest, size_t dest_len, uint32_t src)
  {
    utob(dest, dest_len, (uint64_t) src);
  }

  void utob(void *dest, size_t dest_len, uint16_t src)
  {
    utob(dest, dest_len, (uint64_t) src);
  }

  void itob(void *dest, size_t dest_len, int32_t src)
  {
    itob(dest, dest_len, (int64_t) src);
  }

  void itob(void *dest, size_t dest_len, int16_t src)
  {
    itob(dest, dest_len, (int64_t) src);
  }

  uint64_t btou(void *src, size_t src_len)
  {
    uint64_t retval = 0;

    if (!src_len) { return 0; }
    size_t i = src_len - 1;
    do
    {
      retval = retval << 8;
      retval |= ((uint8_t *) src)[i];
    } while (i--);

    return retval;
  }

  int64_t btoi(void *src, size_t src_len)
  {
    int64_t retval = 0;
    size_t i = sizeof(int64_t);

    if (!src_len) { return 0; }

    /* If array is shorter than int, need to propagate sign bit */
    for (; i >= src_len; --i)
    {
      retval = retval << 8;
      if (((uint8_t *) src)[src_len - 1] & 0x80)
      {  // MSB is set, int is negative
        retval |= 0xff;
      }
    }
    do
    {
      retval = retval << 8;
      retval |= ((uint8_t *) src)[i];
    } while (i--);

    return retval;
  }

  double btof(void *src, size_t src_len, double scale)
  {
    double retval = btoi(src, src_len);
    return retval /= scale;
  }

}; // namespace clearpath

