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
#include <hector_serialization/rosserial/client.h>

namespace hector_serialization {

class serial_client
{
public:
  serial_client(const ros::NodeHandle& node_handle)
    : client_(node_handle, &serial_)
  {
    ros::NodeHandle priv_nh("~");
    priv_nh.param("port", port_, std::string("/dev/ttyUSB0"));
    priv_nh.param("baudrate", baudrate_, 57600);
  }

  ~serial_client() {
    client_.stop();
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

    client_.addDefaultPeer();
    return client_.start();
  }

private:
  SerialPort serial_;
  rosserial::Client client_;

  std::string port_;
  int baudrate_;
};

} // namespace hector_serialization

int main(int argc, char **argv) {
  ros::init(argc, argv, "serial_client");
  ros::NodeHandle node_handle;
  hector_serialization::serial_client serial_client(node_handle);
  if (!serial_client.init()) return 1;
  ros::spin();
  return 0;
}
