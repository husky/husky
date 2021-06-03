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
*  File: Logger.cpp
*  Desc: Provides the Logger singleton which is used within the Clearpath API
*        for log / trace message control
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

#include <iostream>
#include <fstream>
#include <signal.h>
#include <unistd.h>

using namespace std;

#include "husky_base/horizon_legacy/Logger.h"

namespace clearpath
{

  const char *Logger::levelNames[] = {"ERROR", "EXCEPTION", "WARNING", "INFO", "DETAIL"};

  void loggerTermHandler(int signum)
  {
    Logger::instance().close();

    if ((signum == SIGABRT) || (signum == SIGSEGV))
    {
      /* We need to catch these so we can flush out any last messages.
       * having done this, probably the most consistent thing to do
       * is re-raise the signal.  (We certainly don't want to just
       * ignore these) */
      signal(signum, SIG_DFL);
      kill(getpid(), signum);
    }
  }

  Logger &Logger::instance()
  {
    static Logger instance;
    return instance;
  }

  Logger::Logger() :
      enabled(true),
      level(WARNING),
      stream(&cerr)
  {
    nullStream = new ofstream("/dev/null");
  }

  Logger::~Logger()
  {
    close();
  }

  void Logger::close()
  {
    // The actual output stream is owned by somebody else, we only need to flush it
    stream->flush();

    nullStream->close();
    delete nullStream;
    nullStream = 0;
  }

  std::ostream &Logger::entry(enum logLevels msg_level, const char *file, int line)
  {
    if (!enabled) { return *nullStream; }
    if (msg_level > this->level) { return *nullStream; }

    /* Construct the log entry tag */
    // Always the level of the message
    *stream << levelNames[msg_level];
    // If file/line information is provided, need to print it with brackets:
    if (file || (line >= 0))
    {
      *stream << " (";
      if (file) { *stream << file; }
      // Only want a comma if we have both items
      if (file && (line >= 0)) { *stream << ","; }
      if (line >= 0) { *stream << line; }
      *stream << ")";
    }
    *stream << ": ";
    return *stream;
  }

  void Logger::setEnabled(bool en)
  {
    enabled = en;
  }

  void Logger::setLevel(enum logLevels newLevel)
  {
    level = newLevel;
  }

  void Logger::setStream(ostream *newStream)
  {
    stream->flush();
    stream = newStream;
  }

  void Logger::hookFatalSignals()
  {
    signal(SIGINT, loggerTermHandler);
    signal(SIGTERM, loggerTermHandler);
    /* If there's an abort or segfault in Logger.close(), well...
     * we're pretty much totally screwed anyway. */
    signal(SIGABRT, loggerTermHandler);
    signal(SIGSEGV, loggerTermHandler);
  }

} // namespace clearpath
