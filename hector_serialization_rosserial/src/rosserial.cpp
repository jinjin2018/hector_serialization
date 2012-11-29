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

#include <hector_serialization/rosserial.h>
#include <hector_serialization/rosserial/peer.h>
#include <hector_serialization/rosserial/publisher.h>
#include <hector_serialization/rosserial/subscriber.h>
#include <hector_serialization/rosserial/service_server.h>
#include <hector_serialization/rosserial/service_client.h>

namespace hector_serialization {

const std::string RosSerial::DEFAULT_PEER("");

RosSerial::RosSerial(const ros::NodeHandle &node_handle, Mode mode)
  : node_handle_(node_handle)
  , mode_(mode)
  , is_running_(false)
{
  // default parameters
  parameter_.timeout = 5.0;
  parameter_.retries = 3;
  parameter_.sync_interval = 1.0;
  parameter_.max_publishers = 78;
  parameter_.max_subscribers = 78;
  parameter_.output_size = 512;

  ros::NodeHandle priv_nh("~");
  priv_nh.getParam("timeout", parameter_.timeout);
  priv_nh.getParam("retries", parameter_.retries);
  priv_nh.getParam("sync_interval", parameter_.sync_interval);
  priv_nh.getParam("max_publishers", parameter_.max_publishers);
  priv_nh.getParam("max_subscribers", parameter_.max_subscribers);
  priv_nh.getParam("output_size", parameter_.output_size);
}

RosSerial::~RosSerial()
{}

bool RosSerial::start()
{
  boost::recursive_mutex::scoped_lock lock(peers_mutex_);
  for (iterator it = begin(); it != end(); ++it) {
    PeerPtr peer = it->second;
    peer->start();
  }

  is_running_ = true;
  return is_running_;
}

void RosSerial::stop()
{
  boost::recursive_mutex::scoped_lock lock(peers_mutex_);
  for (iterator it = begin(); it != end(); ++it) {
    PeerPtr peer = it->second;
    peer->stop();
  }

  local_publishers_.clear();
  local_subscribers_.clear();
  local_service_servers_.clear();
  local_service_clients_.clear();

  is_running_ = false;
}

void RosSerial::handle(const ConstBuffers1& buffer, const Context &context)
{
  PeerPtr peer;
  {
    boost::recursive_mutex::scoped_lock lock(peers_mutex_);
    peer = getPeer(context);
    if (!peer) peer = getPeer(DEFAULT_PEER);
    if (!peer) peer = addPeer(context);
  }
  peer->handle(buffer, context);
}

PeerPtr RosSerial::getPeer(const Context& context)
{
  return getPeer(context.identifier());
}

PeerPtr RosSerial::getPeer(const std::string& name)
{
  boost::recursive_mutex::scoped_lock lock(peers_mutex_);
  if (!peers_.count(name)) return PeerPtr();
  return peers_[name];
}

PeerPtr RosSerial::addPeer(const Context& context)
{
  return addPeer(context.identifier(), context);
}

PeerPtr RosSerial::addPeer(const std::string& name, const Context& context)
{
  boost::recursive_mutex::scoped_lock lock(peers_mutex_);
  if (peers_.count(name)) return peers_[name];

  ROS_INFO("Adding new peer %s", name.c_str());
  peers_[name].reset(new rosserial::Peer(this, name, context));
  if (is_running_) peers_[name]->start();
  return peers_[name];
}

void RosSerial::removePeer(const std::string &name)
{
  if (name == DEFAULT_PEER) {
    ROS_ERROR("Cannot remove default peer!");
    return;
  }

  boost::recursive_mutex::scoped_lock lock(peers_mutex_);
  if (peers_.count(name) == 0) return;


  ROS_INFO("Removing peer %s", name.c_str());
  peers_[name]->stop();
  peers_.erase(name);
}

PublisherPtr RosSerial::advertise(const TopicInfo &topic_info, const PeerPtr &peer, uint32_t queue_size)
{
  ROS_INFO("Advertising %s [%s] on behalf of %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str(), peer->getName().c_str());
  return PublisherPtr(new rosserial::Publisher(node_handle_, peer, topic_info, queue_size));
}

SubscriberPtr RosSerial::subscribe(const TopicInfo &topic_info, const PeerPtr &peer, uint32_t queue_size)
{
  ROS_INFO("Subscribing %s [%s] on behalf of %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str(), peer->getName().c_str());
  return SubscriberPtr(new rosserial::Subscriber(node_handle_, peer, topic_info, queue_size));
}

ServiceServerPtr RosSerial::advertiseService(const TopicInfo &topic_info, const PeerPtr &peer)
{
  ROS_INFO("Advertising service %s [%s] on behalf of %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str(), peer->getName().c_str());
  return ServiceServerPtr(new rosserial::ServiceServer(node_handle_, peer, topic_info));
}

ServiceClientPtr RosSerial::serviceClient(const TopicInfo &topic_info, const PeerPtr &peer, bool persistent)
{
  ROS_INFO("Subscribing service %s [%s] on behalf of %s", topic_info.topic_name.c_str(), topic_info.message_type.c_str(), peer->getName().c_str());
  return ServiceClientPtr(new rosserial::ServiceClient(node_handle_, peer, topic_info, persistent));
}

std::string RosSerial::ContextElement::identifier() const
{
  return peer->getName() + "/" + boost::lexical_cast<std::string>(topic_info.topic_id);
}

} // namespace hector_serialization
