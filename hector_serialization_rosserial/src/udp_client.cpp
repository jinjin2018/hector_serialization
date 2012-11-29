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
#include <hector_serialization/rosserial/client.h>

namespace hector_serialization {

class udp_client
{
public:
  udp_client(const ros::NodeHandle& node_handle)
    : client_(node_handle, &socket_)
  {
    ros::NodeHandle priv_nh("~");
    priv_nh.param("bind_port", local_port_, 0);
    priv_nh.param("bind_addr", local_addr_, std::string());
    priv_nh.param("remote_port", remote_port_, 8090);
    priv_nh.param("remote_addr", remote_addr_, std::string());

    remote_endpoint_ = socket::Udp::Endpoint(boost::asio::ip::address::from_string(remote_addr_), remote_port_);
    local_endpoint_ = socket::Udp::Endpoint(remote_endpoint_.protocol(), local_port_);
    if (!local_addr_.empty()) local_endpoint_.address(boost::asio::ip::address::from_string(local_addr_));
  }

  ~udp_client() {
    client_.stop();
  }

  bool init() {
    try {
      socket_.open(remote_endpoint_.protocol());
      socket_.bind(local_endpoint_);
      socket_.connect(remote_endpoint_);

    } catch(boost::system::system_error&  e) {
      ROS_ERROR("%s", e.what());
      throw;
    }

    client_.addDefaultPeer();
    return client_.start();
  }

private:
  socket::Udp socket_;
  rosserial::Client client_;

  std::string local_addr_;
  int local_port_;
  std::string remote_addr_;
  int remote_port_;
  socket::Udp::Endpoint local_endpoint_;
  socket::Udp::Endpoint remote_endpoint_;
};

} // namespace hector_serialization

int main(int argc, char **argv) {
  ros::init(argc, argv, "udp_client");
  ros::NodeHandle node_handle;
  hector_serialization::udp_client udp_client(node_handle);
  if (!udp_client.init()) return 1;
  ros::spin();
  return 0;
}
