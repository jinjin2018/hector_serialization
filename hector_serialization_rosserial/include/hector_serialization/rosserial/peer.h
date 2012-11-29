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

#ifndef HECTOR_SERIALIZATION_ROSSERIAL_PEER_H
#define HECTOR_SERIALIZATION_ROSSERIAL_PEER_H

#include <hector_serialization/rosserial.h>

namespace hector_serialization {
namespace rosserial {

class Peer : public boost::enable_shared_from_this<Peer> {
public:
  Peer(RosSerial *owner, const std::string& name, const Context& context);
  virtual ~Peer();

  bool start();
  void stop();

  std::string getName() const { return name_; }
  const Context& getContext() const { return context_; }
  bool connected() const { return connected_; }

  void handle(const ConstBuffer &buffer, const Context &context);
  bool write(const ConstBuffer &message, const TopicInfo &topic_info);
  bool write(const ConstBuffer &message, uint16_t topic_id);

  // server functions
  void requestTopics();

  // client functions
  void negotiateTopics();
  bool synchronizeTime(bool wait = false);
  ros::Duration getTimeOffset() const { return time_offset_; }
  ros::Time now() const { return ros::Time::now() + getTimeOffset(); }

private:
  void receiveCallback(const ConstBuffer& buffer, RosSerial::Protocol::ReceiveHeader const &header, const Context &context);
  void handleTimeRequest(const ConstBuffer &payload);
  void handleTimeResponse(const ConstBuffer &payload);
  void handleLog(const ConstBuffer &payload);
  bool handleParameterRequest(const ConstBuffer &payload);
  bool handleMessage(const ConstBuffer &message, uint16_t topic_id);

  uint16_t generateTopicId();
  std::string& remapName(std::string& name);

private:
  RosSerial *owner_;
  std::string name_;
  Context context_;
  bool connected_;
  int retry_counter_;

  StreamBuf in_;

  ros::Time last_sync_time_;
  ros::Time last_received_;
  boost::mutex time_synchronize_mutex_;
  boost::condition time_synchronized_condition_;
  ros::Duration time_offset_;

  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent &);

  std::map<TopicInfo::_topic_id_type,PublisherPtr> remote_publishers_;
  std::map<TopicInfo::_topic_id_type,SubscriberPtr> remote_subscribers_;
  std::map<TopicInfo::_topic_id_type,ServiceServerPtr> remote_service_servers_;
  std::map<TopicInfo::_topic_id_type,ServiceClientPtr> remote_service_clients_;
};

} // namespace rosserial
} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_ROSSERIAL_PEER_H
