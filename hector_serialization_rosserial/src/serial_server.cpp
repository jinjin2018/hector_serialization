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
//=================================================================================================

#include <ros/ros.h>
#include <hector_serialization/device/serial_port.h>
#include <hector_serialization/rosserial/server.h>

namespace hector_serialization {

class serial_server
{
public:
  serial_server(const ros::NodeHandle& node_handle)
    : server_(node_handle, &serial_)
  {
    ros::NodeHandle priv_nh("~");
    priv_nh.param("port", port_, std::string("/dev/ttyUSB0"));
    priv_nh.param("baudrate", baudrate_, 57600);
  }

  ~serial_server() {
    server_.stop();
  }

  bool init() {
    try {
      serial_.setDevice(port_);
      serial_.open();
      serial_.setBaudrate(baudrate_);

    } catch(boost::system::system_error&  e) {
      ROS_ERROR("%s", e.what());
      throw;
    }

    server_.addDefaultPeer();
    return server_.start();
  }

private:
  SerialPort serial_;
  rosserial::Server server_;

  std::string port_;
  int baudrate_;
};

} // namespace hector_serialization

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_server");
  ros::NodeHandle node_handle;
  hector_serialization::serial_server serial_server(node_handle);
  if (!serial_server.init()) return 1;
  ros::spin();
  return 0;
}
