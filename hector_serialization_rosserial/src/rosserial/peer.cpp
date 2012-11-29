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

#include <hector_serialization/rosserial/peer.h>
#include <hector_serialization/rosserial/publisher.h>
#include <hector_serialization/rosserial/subscriber.h>
#include <hector_serialization/rosserial/service_server.h>
#include <hector_serialization/rosserial/service_client.h>
#include <hector_serialization/serialization.h>

#include <rosserial_msgs/Log.h>
#include <rosserial_msgs/RequestParam.h>
#include <std_msgs/Time.h>

namespace hector_serialization {
namespace rosserial {

Peer::Peer(RosSerial *owner, const std::string &name, const Context& context)
  : owner_(owner)
  , name_(name)
  , context_(context)
  , connected_(false)
  , retry_counter_(0)
{
  timer_ = owner->getNodeHandle().createTimer(ros::Duration(owner->parameter().sync_interval), &Peer::timerCallback, this);
}

Peer::~Peer()
{
  stop();
}

bool Peer::start()
{
  if (owner_->getMode() & RosSerial::SERVER) {
    requestTopics();
  }

  if (owner_->getMode() & RosSerial::CLIENT) {
    synchronizeTime();
  }

  timer_.start();
  return true;
}

void Peer::stop()
{
  timer_.stop();
  remote_publishers_.clear();
  remote_subscribers_.clear();
}

void Peer::handle(const ConstBuffer &buffer, const Context &context)
{
  in_.commit(copy(buffer, in_.prepare(buffer_size(buffer))));
  RosSerial::Protocol::ReceiveHeader header;

  for(std::size_t size = in_.size(); size > 0; in_.consume(1), size = in_.size()) {
    uint8_t const *data = boost::asio::buffer_cast<uint8_t const *>(in_.data());
    if (data[0] != 0xFF) { in_.consume(1); continue; }
    if (size < 7u) break;
    if (data[1] != 0xFF) { in_.consume(1); continue; }
    header.topic_id = data[2] | (data[3] << 8);
    header.length = data[4] | (data[5] << 8);
    if (size < 7u + header.length) break;
    uint8_t checksum = 0;
    for(uint8_t const *temp = &data[2]; temp < &data[6u + header.length]; ++temp) checksum += *temp;
    if (255 - checksum == data[6u + header.length]) {
      receiveCallback(ConstBuffer(data + 6, header.length), header, context);
    } else {
      ROS_ERROR("rosserial: Checksum error on peer %s (%02x != %02x)", getName().c_str(), 255 - checksum, data[6 + header.length]);
    }
    in_.consume(6u + header.length); // +1 from for loop
  }
}

void Peer::receiveCallback(const ConstBuffer& payload, RosSerial::Protocol::ReceiveHeader const &header, const Context &context)
{
  TopicInfo topic_info;

  switch(header.topic_id) {
    case TopicInfo::ID_PUBLISHER:
      if (header.length == 0) {
        negotiateTopics();
        break;
      }

      try {
        deserialize(payload, topic_info);
      } catch(std::runtime_error &e) {
        ROS_WARN("Failed to deserialize TopicInfo for ID_PUBLISHER message from peer %s: %s", getName().c_str(), e.what());
        break;
      }

      if (remote_publishers_.count(topic_info.topic_id) > 0) break;
      remapName(topic_info.topic_name);
      remote_publishers_[topic_info.topic_id] = owner_->advertise(topic_info, shared_from_this());
      break;

    case TopicInfo::ID_SUBSCRIBER:
      try {
        deserialize(payload, topic_info);
      } catch(std::runtime_error &e) {
        ROS_WARN("Failed to deserialize TopicInfo for ID_SUBSCRIBER message from peer %s: %s", getName().c_str(), e.what());
        break;
      }
      if (remote_subscribers_.count(topic_info.topic_id) > 0) break;
      remapName(topic_info.topic_name);
      remote_subscribers_[topic_info.topic_id] = owner_->subscribe(topic_info, shared_from_this());
      break;

    case TopicInfo::ID_SERVICE_SERVER:
      try {
        deserialize(payload, topic_info);
      } catch(std::runtime_error &e) {
        ROS_WARN("Failed to deserialize TopicInfo for ID_SERVICE_SERVER message from peer %s: %s", getName().c_str(), e.what());
        break;
      }
      if (remote_service_servers_.count(topic_info.topic_id) > 0) break;
      remapName(topic_info.topic_name);
      remote_service_servers_[topic_info.topic_id] = owner_->advertiseService(topic_info, shared_from_this());
      break;

    case TopicInfo::ID_SERVICE_CLIENT:
      try {
        deserialize(payload, topic_info);
      } catch(std::runtime_error &e) {
        ROS_WARN("Failed to deserialize TopicInfo for ID_SERVICE_CLIENT message from peer %s: %s", getName().c_str(), e.what());
        break;
      }
      if (remote_service_clients_.count(topic_info.topic_id) > 0) break;
      remapName(topic_info.topic_name);
      remote_service_clients_[topic_info.topic_id] = owner_->serviceClient(topic_info, shared_from_this());
      break;

    case TopicInfo::ID_PARAMETER_REQUEST:
      if (owner_->getMode() & RosSerial::SERVER)
        handleParameterRequest(payload);
      break;

    case TopicInfo::ID_LOG:
      handleLog(payload);
      break;

    case TopicInfo::ID_TIME:
      if (owner_->getMode() & RosSerial::SERVER)
        handleTimeRequest(payload);
      if (owner_->getMode() & RosSerial::CLIENT)
        handleTimeResponse(payload);
      break;

    default:
      handleMessage(payload, header.topic_id);
      break;
  }

  last_received_ = ros::Time::now();
}

void Peer::requestTopics()
{
  write(EmptyBuffer(), 0);
  last_received_ = ros::Time::now();
}

void Peer::negotiateTopics()
{

}

bool Peer::synchronizeTime(bool wait)
{
  boost::mutex::scoped_try_lock lock(time_synchronize_mutex_);
  if (!lock) return false;

  // send empty timestamp message
  std_msgs::Time now;
  now.data = ros::Time::now();
  uint8_t message[serializationLength(now)];
  serialize(MutableBuffer(message, sizeof(message)), now);
  write(ConstBuffer(message, sizeof(message)), TopicInfo::ID_TIME);
  this->last_sync_time_ = now.data;

  if (!wait) {
    lock.release(); // releases ownership of the lock, mutex is still locked
    return true;
  }

  if (!time_synchronized_condition_.timed_wait(lock, ros::Duration(owner_->parameter().timeout).toBoost())) return false;
  return !last_sync_time_.isZero();
}

void Peer::handleLog(const ConstBuffer &payload)
{
  rosserial_msgs::Log log;
  try {
    deserialize(payload, log);
  } catch(std::runtime_error &e) {
    ROS_WARN("Failed to deserialize Log message from peer %s: %s", getName().c_str(), e.what());
    return;
  }

  switch(log.level) {
    case rosserial_msgs::Log::DEBUG: ROS_DEBUG_STREAM_NAMED(getName(), log.msg); break;
    case rosserial_msgs::Log::INFO:  ROS_INFO_STREAM_NAMED (getName(), log.msg); break;
    case rosserial_msgs::Log::WARN:  ROS_WARN_STREAM_NAMED (getName(), log.msg); break;
    case rosserial_msgs::Log::ERROR: ROS_ERROR_STREAM_NAMED(getName(), log.msg); break;
    case rosserial_msgs::Log::FATAL: ROS_FATAL_STREAM_NAMED(getName(), log.msg); break;
  }
}

void Peer::handleTimeRequest(const ConstBuffer &payload)
{
  // send answer with current timestamp
  std_msgs::Time now;
  now.data = ros::Time::now();
  uint8_t message[serializationLength(now)];
  serialize(MutableBuffer(message, sizeof(message)), now);
  write(ConstBuffer(message, sizeof(message)), TopicInfo::ID_TIME);

  last_sync_time_ = now.data;
  retry_counter_ = 0;
}

void Peer::handleTimeResponse(const ConstBuffer &payload)
{
  if (last_sync_time_.isZero()) {
    ROS_WARN("Received time response from peer %s without having sent a request", getName().c_str());
    time_synchronize_mutex_.unlock();
    return;
  }

  // calculate time offset
  std_msgs::Time response;
  ros::Time now = ros::Time::now();
  try {
    deserialize(payload, response);
  } catch(std::runtime_error &e) {
    ROS_WARN("Failed to deserialize Time message from peer %s: %s", getName().c_str(), e.what());
    time_synchronize_mutex_.unlock();
    last_sync_time_ = ros::Time();
    return;
  }

  // successfully synced time
  time_offset_ = response.data + (now - last_sync_time_) * 0.5 - now;
  retry_counter_ = 0;

  time_synchronized_condition_.notify_all();
  time_synchronize_mutex_.unlock();
}

bool Peer::handleParameterRequest(const ConstBuffer &payload)
{
  rosserial_msgs::RequestParamRequest req;
  try {
    deserialize(payload, req);
  } catch(std::runtime_error &e) {
    ROS_WARN("Failed to deserialize ParameterRequest message from peer %s: %s", getName().c_str(), e.what());
    return false;
  }

  remapName(req.name);

  XmlRpc::XmlRpcValue value;
  std::vector<XmlRpc::XmlRpcValue> param;

  if (!owner_->getNodeHandle().getParam(req.name, value)) {
    ROS_ERROR("Parameter %s does not exist", req.name.c_str());
    return false;
  }

  if (value.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_ERROR("Cannot send param %s because it is a struct", req.name.c_str());
    return false;
  }

  if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    param.push_back(value);
  } else {
    param.insert(param.end(), &value[0], &value[value.size()]);
  }

  XmlRpc::XmlRpcValue::Type t = param.size() > 0 ? param[0].getType() : XmlRpc::XmlRpcValue::TypeInvalid;
  rosserial_msgs::RequestParamResponse resp;
  for(std::vector<XmlRpc::XmlRpcValue>::iterator it = param.begin(); it != param.end(); ++it) {
    if (t != it->getType()) {
      ROS_ERROR("All Paramers in the list %s must be of the same type", req.name.c_str());
      return false;
    }

    if (t == XmlRpc::XmlRpcValue::TypeInt)
      resp.ints.push_back(*it);
    else if (t == XmlRpc::XmlRpcValue::TypeDouble)
      resp.floats.push_back(static_cast<double>(*it));
    else if (t == XmlRpc::XmlRpcValue::TypeString)
      resp.strings.push_back(*it);
  }

  return write(serialize(resp), TopicInfo::ID_PARAMETER_REQUEST);
}

bool Peer::handleMessage(const ConstBuffer &message, uint16_t topic_id)
{
  if (remote_publishers_.count(topic_id) > 0) {
    remote_publishers_[topic_id]->publish(message);
    return true;
  }

  if (remote_service_clients_.count(topic_id) > 0) {
    remote_service_clients_[topic_id]->call(message);
    return true;
  }

  if (remote_service_servers_.count(topic_id) > 0) {
    remote_service_servers_[topic_id]->handleResponse(message);
    return true;
  }

  ROS_DEBUG("Received message with unknown id %u from peer %s", topic_id, getName().c_str());
  requestTopics();

  return false;
}

void Peer::timerCallback(const ros::TimerEvent &)
{
  ros::Time now = ros::Time::now();

  if (owner_->getMode() & RosSerial::CLIENT) synchronizeTime();

  if (now - last_received_ > ros::Duration(owner_->parameter().timeout)) {
    if (++retry_counter_ > owner_->parameter().retries) {
      PeerPtr this_shared = shared_from_this();
      ROS_ERROR("Lost sync with peer %s, disabling peer...", getName().c_str());
      owner_->removePeer(getName());
      retry_counter_ = 0;
      return;
    }

    ROS_WARN("Lost sync with peer %s, restarting (retry %d)...", getName().c_str(), retry_counter_);

    if (owner_->getMode() & RosSerial::SERVER) {
      requestTopics();
    }

    if (owner_->getMode() & RosSerial::CLIENT) {
      negotiateTopics();
      time_synchronize_mutex_.unlock();
      synchronizeTime();
    }
  }
}

uint16_t Peer::generateTopicId()
{
  return 0;
}

std::string& Peer::remapName(std::string& name)
{
  std::size_t pos = name.find("%");
  if (pos != std::string::npos) {
    if (getName() != RosSerial::DEFAULT_PEER) {
      name.replace(pos, 1, getName());
    } else {
      if (name.substr(pos, 2) == "%/")
        name.replace(pos, 2, "");
      else
        name.replace(pos, 1, "");
    }
  }

  return name;
}

bool Peer::write(const ConstBuffer &message, const TopicInfo& topic_info)
{
  return write(message, topic_info.topic_id);
}

bool Peer::write(const ConstBuffer &message, uint16_t topic_id)
{
  boost::array<uint8_t,6> header;
  uint16_t length = buffer_size(message);

  header[0] = 0xFF;
  header[1] = 0xFF;
  header[2] = topic_id & 0xFF;
  header[3] = (topic_id >> 8) & 0xFF;
  header[4] = length & 0xFF;
  header[5] = (length >> 8) & 0xFF;

  uint8_t checksum = 255 - header[2] - header[3] - header[4] - header[5];
  for(buffers_iterator<ConstBuffer> it = buffers_begin(message); it != buffers_end(message); ++it) checksum -= *it;

  return owner_->write(ConstBuffer(header.data(), header.size()) + message + ConstBuffer(&checksum, 1), context_);
}

} // namespace rosserial
} // namespace hector_serialization
