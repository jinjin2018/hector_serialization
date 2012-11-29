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

#include <hector_serialization/socket/udp.h>
#include <hector_serialization/serialization.h>

#include <rosserial_msgs/TopicInfo.h>

using namespace hector_serialization;

int main(int argc, char **argv) {
  unsigned int listen_port = 3000, send_port = 3000;
  if (argc > 1) listen_port = boost::lexical_cast<unsigned int>(argv[1]);
  if (argc > 2) send_port = boost::lexical_cast<unsigned int>(argv[2]);

  socket::Udp socket;
  socket::Udp::Endpoint local_endpoint(boost::asio::ip::address_v4::loopback(), listen_port);
  socket::Udp::Endpoint remote_endpoint(boost::asio::ip::address_v4::loopback(), send_port);
  socket.open(local_endpoint.protocol());
  socket.setOption(boost::asio::socket_base::broadcast(true));
  socket.setOption(boost::asio::socket_base::reuse_address(true));
  socket.bind(local_endpoint);
  // socket.connect(socket::Udp::Endpoint(boost::asio::ip::address_v4::loopback(), send_port));

  socket.wait(1.0);

  rosserial_msgs::TopicInfo topic_info;
  topic_info.message_type = "rosserial_msgs/TopicInfo";
  if (socket.write(serialize(topic_info), socket::Udp::ContextElement(remote_endpoint))) {
    std::cout << "Sent: " << std::endl << topic_info << std::endl;
  }

  while (socket.wait(10.0) && socket.size() > 0) {
    deserialize(socket, topic_info);
    std::cout << "Received: " << std::endl << topic_info << std::endl;
  }

  socket.disconnect();
}
