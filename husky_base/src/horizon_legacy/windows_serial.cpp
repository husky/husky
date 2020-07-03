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
*  File: windows_serial.cpp
*  Desc: Windows-compatible serial commands for linking with generic functions
*        defined in serial.h
*  Auth: C. Iverach-Brereton
*
*  Copyright (c) 2020, Clearpath Robotics, Inc.
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

#if !defined(WINDOWS_SERIAL_H) && defined(win_x86)
#define WINDOWS_SERIAL_H

#include "husky_base/horizon_legacy/serial.h"  /* Std. function protos */
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>  /* Malloc */
#include <assert.h>
#include <windows.h>

///////////////////////////////////////////////////////////////////////////////////
//                                                                               //
// TODO: rewrite the bodies of all of these functions to work on Windows!        //
//                                                                               //
///////////////////////////////////////////////////////////////////////////////////
// see: https://stackoverflow.com/questions/15794422/serial-port-rs-232-connection-in-c


// Open the serial port, return the file descriptor for it
int OpenSerial(void **handle, const char *port_name)
{
  HANDLE serialHandle = CreateFile(port_name, GENERIC_READ, GENERIC_WRITE, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);

  // copy the handle back to the provided pointer
  *handle = (void*)malloc(sizeof(serialHandle));
  memcpy(&serialHandle, *handle, sizeof(serialHandle));

  return 0;
}

// Configure the serial port for 115200 baud 8N1
int SetupSerial(void *handle)
{
  // Do some basic settings
  DCB serialParams = { 0 };
  serialParams.DCBlength = sizeof(serialParams);

  GetCommState(serialHandle, &serialParams);
  serialParams.BaudRate = baudrate;
  serialParams.ByteSize = byteSize;
  serialParams.StopBits = stopBits;
  serialParams.Parity = parity;
  SetCommState(serialHandle, &serialParams);

  // Set timeouts
  COMMTIMEOUTS timeout = { 0 };
  timeout.ReadIntervalTimeout = 50;
  timeout.ReadTotalTimeoutConstant = 50;
  timeout.ReadTotalTimeoutMultiplier = 50;
  timeout.WriteTotalTimeoutConstant = 50;
  timeout.WriteTotalTimeoutMultiplier = 10;

  SetCommTimeouts(*handle, &timeout);

  return 0;
}

// write data to the serial port, return the number of bytes written
int WriteData(void *handle, const char *buffer, int length)
{
  DWORD nBytesWritten = 0;
  WriteFile(*handle, buffer, length, &nBytesWritten, NULL);

  if(nBytesWritten != length)
  {
    fprintf(stderr, "Error in serial write\r\n");
    return -1;
  }

  // serial port output monitor
//#define TX_DEBUG
#ifdef TX_DEBUG
  printf("TX:");
  int i;
  for (i=0; i<length; ++i)
    printf(" %x", (unsigned char)(buffer[i]));
  printf("\r\n");
#endif

  return n;
}

// read data from the serial port, return the number of bytes read
int ReadData(void *handle, char *buffer, int length)
{
  DWORD nBytesRead = 0;
  ReadFile(*handle, buffer, length, &nBytesRead, NULL);

  if(nBytesRead <= 0)
  {
    return 0;
  }

  // serial port input monitor
//#define RX_DEBUG
#ifdef RX_DEBUG
  printf("RX:");
  int i;
  for (i=0; i<nBytesRead; ++i)
    printf(" %x", (unsigned char)buffer[i]);
  printf("\r\n");
#endif

  return nBytesRead;
}

// close the serial port, always return 0
int CloseSerial(void *handle)
{
  if (handle == NULL)
    return 0;

  close(*handle);
  free(*handle);
  return 0;
}

#endif // WINDOWS_SERIAL
