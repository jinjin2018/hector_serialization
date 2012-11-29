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

#ifndef HECTOR_SERIALIZATION_PROTOCOL_ROSSERIAL_H
#define HECTOR_SERIALIZATION_PROTOCOL_ROSSERIAL_H

#include <hector_serialization/protocol.h>
#include <hector_serialization/channel.h>
#include <hector_serialization/context.h>
#include <hector_serialization/rosserial/topic_info.h>
#include <ros/node_handle.h>

#include <rosserial_msgs/RequestParam.h>

#include <boost/thread/recursive_mutex.hpp>

namespace hector_serialization {

class RosSerial;

namespace rosserial {
  class Peer;
  class Publisher;
  class Subscriber;
  class ServiceServer;
  class ServiceClient;
}
typedef boost::shared_ptr<rosserial::Peer> PeerPtr;
typedef boost::shared_ptr<rosserial::Publisher> PublisherPtr;
typedef boost::shared_ptr<rosserial::Subscriber> SubscriberPtr;
typedef boost::shared_ptr<rosserial::ServiceServer> ServiceServerPtr;
typedef boost::shared_ptr<rosserial::ServiceClient> ServiceClientPtr;

template <>
class ProtocolInfo<RosSerial> {
public:
  struct Address {};
  struct Status {};

  struct Header
  {
    uint16_t topic_id;
    uint16_t length;
    Header() : topic_id(), length() {}
    bool operator==(const Header& other) const { return topic_id == other.topic_id; }
  };
  typedef Header SendHeader;
  typedef Header ReceiveHeader;

  struct ContextElement : public Context::Element<ContextElement> {
    PeerPtr peer;
    TopicInfo topic_info;

    ContextElement() {}
    ContextElement(PeerPtr peer, TopicInfo topic_info) : peer(peer), topic_info(topic_info) {}

    bool isEqual(const ContextElement &other) const {
      return (other.peer == this->peer) && (other.topic_info.topic_id == this->topic_info.topic_id);
    }

    std::string identifier() const;
  };
};

class RosSerial : public Protocol_<RosSerial>, public ChannelElement
{
public:
  typedef Protocol_<RosSerial> Protocol;
  typedef std::map<std::string,PeerPtr> Peers;
  typedef Peers::iterator iterator;
  static const std::string DEFAULT_PEER;

  enum Mode { SERVER = 1, CLIENT = 2 };
  struct Parameters {
    double timeout;
    int retries;

    double sync_interval;
    int max_subscribers;
    int max_publishers;
    int output_size;
  };

  RosSerial(const ros::NodeHandle &node_handle = ros::NodeHandle(), Mode mode = SERVER);
  virtual ~RosSerial();

  bool start();
  void stop();

  // virtual bool write(const BufferSequence &sequence, const Context& context = Context()) { return false; }
  virtual bool wait(const boost::posix_time::time_duration& timeout) { return false; }
  virtual void handle(const ConstBuffer& buffer, const Context &context);

  PublisherPtr advertise(const TopicInfo& topic_info, const PeerPtr &peer, uint32_t queue_size = 1);
  SubscriberPtr subscribe(const TopicInfo& topic_info, const PeerPtr &peer, uint32_t queue_size = 1);
  ServiceServerPtr advertiseService(const TopicInfo &topic_info, const PeerPtr &peer);
  ServiceClientPtr serviceClient(const TopicInfo &topic_info, const PeerPtr &peer, bool persistent = false);

  Peers getPeers() const { return peers_; }
  PeerPtr getPeer(const Context& context);
  PeerPtr getPeer(const std::string& name);
  PeerPtr addPeer(const std::string& name, const Context& context = Context());
  PeerPtr addPeer(const Context& context);
  void removePeer(const std::string& name);

  ros::NodeHandle &getNodeHandle() { return node_handle_; }
  Mode getMode() { return mode_; }
  const Parameters &parameter() { return parameter_; }

private:
  ros::NodeHandle node_handle_;
  Mode mode_;
  Parameters parameter_;

  bool is_running_;

  boost::recursive_mutex peers_mutex_;
  std::map<std::string,PeerPtr> peers_;
  iterator begin() { return peers_.begin(); }
  iterator end()   { return peers_.end(); }

  std::map<TopicInfo::_topic_id_type,PublisherPtr> local_publishers_;
  std::map<TopicInfo::_topic_id_type,SubscriberPtr> local_subscribers_;
  std::map<TopicInfo::_topic_id_type,ServiceServerPtr> local_service_servers_;
  std::map<TopicInfo::_topic_id_type,ServiceClientPtr> local_service_clients_;
};

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_PROTOCOL_ROSSERIAL_H
