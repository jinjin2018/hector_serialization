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
#include <hector_serialization/socket/udp.h>
#include <hector_serialization/rosserial/server.h>

namespace hector_serialization {

class udp_server
{
public:
  udp_server(const ros::NodeHandle& node_handle)
    : server_(node_handle, &socket_)
  {
    ros::NodeHandle priv_nh("~");
    priv_nh.param("bind_address", local_addr_, std::string());
    priv_nh.param("port", local_port_, 8090);

    bool ipv6;
    priv_nh.param("ipv6", ipv6, false);
    if (ipv6)
      local_endpoint_ = socket::Udp::Endpoint(boost::asio::ip::udp::v6(), local_port_);
    else
      local_endpoint_ = socket::Udp::Endpoint(boost::asio::ip::udp::v4(), local_port_);
    if (!local_addr_.empty()) local_endpoint_.address(boost::asio::ip::address::address::from_string("aaaa::c30c:0:0:ffff"));
  }

  ~udp_server() {
    server_.stop();
  }

  bool init() {
    try {
      socket_.open(local_endpoint_.protocol());
      socket_.bind(local_endpoint_);

    } catch(boost::system::system_error&  e) {
      ROS_ERROR("%s", e.what());
      throw;
    }

    return server_.start();
  }

private:
  socket::Udp socket_;
  rosserial::Server server_;

  std::string local_addr_;
  int local_port_;
  socket::Udp::Endpoint local_endpoint_;
};

} // namespace hector_serialization

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_server");
  ros::NodeHandle node_handle;
  hector_serialization::udp_server udp_server(node_handle);
  if (!udp_server.init()) return 1;
  ros::spin();
  return 0;
}
