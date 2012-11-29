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

#include <hector_serialization/rosserial/service_server.h>
#include <hector_serialization/rosserial/peer.h>
#include <hector_serialization/serialization.h>

namespace hector_serialization {
namespace rosserial {

static const double DEFAULT_TIMEOUT = 5.0;

ServiceServer::ServiceServer(ros::NodeHandle node_handle, const boost::shared_ptr<Peer>& peer, const TopicInfo& service_info)
  : node_handle_(node_handle)
  , peer_(peer)
  , service_info_(service_info)
  , timeout_(DEFAULT_TIMEOUT)
{
}

void ServiceServer::handleServicePublisher(const TopicInfo& topic_info)
{

}

void ServiceServer::handleServiceSubscriber(const TopicInfo& topic_info)
{

}

void ServiceServer::advertise()
{
  ros::AdvertiseServiceOptions avo;
  avo.init(service_info_.topic_name, boost::function<bool(MessageInstance &request, MessageInstance &response)>(boost::bind(&ServiceServer::callback, this, _1, _2)));
  avo.md5sum = service_info_.md5sum;
  avo.datatype = service_info_.message_type;
  avo.req_datatype = request_topic_info_.message_type;
  avo.res_datatype = response_topic_info_.message_type;
  static_cast<ros::ServiceServer&>(*this) = node_handle_.advertiseService(avo);
}

bool ServiceServer::callback(MessageInstance &request, MessageInstance &response)
{
  if (request.getMD5Sum() != request_topic_info_.md5sum) {
    ROS_WARN("Received a service request of type [%s/%s] for a service with request type [%s/%s]",
             request.getDataType().c_str(), request.getMD5Sum().c_str(),
             request_topic_info_.message_type.c_str(), request_topic_info_.md5sum.c_str());
    return false;
  }

  boost::mutex::scoped_lock request_lock(request_mutex_);
  boost::mutex::scoped_lock response_lock(response_mutex_);

  // send request to the peer
  peer_->write(request.getMessage(), request_topic_info_);

  if (!response_condition_.timed_wait(response_lock, timeout_.toBoost())) {
    ROS_ERROR("Service call of type [%s/%s] for peer %s failed. No answer from peer within %f seconds.",  service_info_.message_type.c_str(), service_info_.md5sum.c_str(), peer_->getName().c_str(), timeout_.toSec());
    return false;
  }

  // fill answer
  response = response_message_;
  return true;
}

void ServiceServer::handleResponse(const ConstBuffer& response)
{
  boost::mutex::scoped_try_lock request_lock(request_mutex_);
  if (request_lock.owns_lock()) {
    ROS_ERROR("Received a response for service call of type [%s/%s] from peer %s without having sent a request.", service_info_.message_type.c_str(), service_info_.md5sum.c_str(), peer_->getName().c_str());
    return;
  }

  boost::mutex::scoped_lock response_lock(response_mutex_);
  response_message_.copy(response);
  response_condition_.notify_one();
}

} // namespace rosserial
} // namespace hector_serialization
