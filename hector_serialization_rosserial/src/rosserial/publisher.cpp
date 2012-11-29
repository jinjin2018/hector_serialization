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

#include <hector_serialization/rosserial/publisher.h>
#include <hector_serialization/rosserial/message_instance.h>
#include <hector_serialization/rosserial/peer.h>
#include <hector_serialization/buffer.h>

namespace hector_serialization {
namespace rosserial {

Publisher::Publisher(ros::NodeHandle node_handle, const boost::shared_ptr<Peer> &peer, const TopicInfo& topic_info, uint32_t queue_size)
  : peer_(peer)
  , topic_info_(topic_info)
{
  ros::AdvertiseOptions avo(topic_info.topic_name, queue_size, topic_info.md5sum, topic_info.message_type, std::string());
  static_cast<ros::Publisher&>(*this) = node_handle.advertise(avo);
}

void Publisher::publish(const ConstBuffer& message)
{
  ROS_DEBUG("Publishing a message on topic %s from peer %s (%s)", getTopic().c_str(), peer_->getName().c_str(), peer_->getContext().to_string().c_str());
  ros::Publisher::publish(MessageInstance(message, topic_info_));
}

} // namespace rosserial
} // namespace hector_serialization
