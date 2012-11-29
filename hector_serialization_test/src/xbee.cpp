//=================================================================================================
// Copyright (c) 2012, Johannes Meyer, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//==========================================

#include <hector_serialization/device/serial_port.h>
#include <hector_serialization/device/file.h>
#include <hector_serialization/xbee.h>
#include <hector_serialization/serialization.h>

#include <rosgraph_msgs/Clock.h>

#include <linux/serial.h>

using namespace hector_serialization;

void statusCallback(const XBee::Status& status) {
  std::cout << "Received XBee status " << int(status.status) << std::endl;
}

int main(int argc, char **argv)
{
  SerialPort serial_port(argc > 1 ? std::string(argv[1]) : std::string("/dev/ttyUSB0"));
  try {
    serial_port.open();
    serial_port.setBaudrate(115200);

//    struct serial_struct ser;
//    ioctl (serial_port.native(), TIOCGSERIAL, &ser);
//    // set custom divisor
//    ser.custom_divisor = ser.baud_base / 111111;
//    // update flags
//    ser.flags &= ~ASYNC_SPD_MASK;
//    ser.flags |= ASYNC_SPD_CUST;

//    if (ioctl (serial_port.native(), TIOCSSERIAL, &ser) < 0)
//    {
//      std::cerr << std::string(strerror(errno)) << std::endl;
//    }
  } catch(std::runtime_error& e) {
    std::cerr << e.what() << std::endl;
    return 1;
  }

  OutputFile file("tmp.bin");
  file.open();

  rosgraph_msgs::Clock clock;
//  serial_port->write(serialize(clock));
//  file.write(serialize(clock));

  XBee xbee(&serial_port);
  xbee.addOutput(&file);
  if (!xbee.configure()) return 2;

  xbee.addStatusCallback(XBee::StatusCallback(&statusCallback));
  xbee.header(XBee::broadcast());
  xbee.write(serialize(clock));

  pause();

  return 0;
}
