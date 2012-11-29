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

#ifndef HECTOR_SERIALIZATION_ROSSERIAL_MESSAGE_INSTANCE_H
#define HECTOR_SERIALIZATION_ROSSERIAL_MESSAGE_INSTANCE_H

#include <hector_serialization/buffer.h>
#include <hector_serialization/serialization.h>
#include <hector_serialization/rosserial/topic_info.h>

#include <ros/message_traits.h>
#include <ros/service_traits.h>
#include <ros/advertise_options.h>
#include <ros/advertise_service_options.h>
#include <ros/subscription_callback_helper.h>

namespace hector_serialization {
namespace rosserial {

class MessageInstance
{
public:
  MessageInstance();
  MessageInstance(const MessageInstance &other);
  MessageInstance(const TopicInfo &topic_info);
  MessageInstance(const ConstBuffer &message, const TopicInfo &topic_info);
  ~MessageInstance();

  MessageInstance &operator=(const ConstBuffer &message);

  std::string const& getTopic()             const;
  std::string const& getDataType()          const;
  std::string const& getMD5Sum()            const;
  std::string const& getMessageDefinition() const;

  ConstBuffer const& getMessage() const;

  void morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def,
             const std::string& latching);

  //! Test whether the underlying message of the specified type.
  /*!
   * returns true iff the message is of the template type
   */
  template<class T>
  bool isType() const;

  //! Templated call to instantiate a message
  /*!
   * returns NULL pointer if incompatible
   */
  template<class T>
  boost::shared_ptr<T> instantiate() const;

  //! Write serialized message contents out to a stream
  template<typename Stream>
  void write(Stream& stream) const;

  template<typename Stream>
  void read(Stream& stream);

  //! Size of serialized message
  uint32_t size() const;

  //! Pointer to the data of ther serialized message
  const uint8_t *data() const;

  void copy(const ConstBuffer &source);

private:
  ConstBuffer message_;
  TopicInfo topic_info_;
  std::string message_definition_, latching_;

  uint8_t *buffer_;
  std::size_t buffer_alloc_;
};

template<class T>
bool MessageInstance::isType() const {
  char const* md5sum = ros::message_traits::MD5Sum<T>::value();
  return md5sum == std::string("*") || md5sum == getMD5Sum();
}

template<class T>
boost::shared_ptr<T> MessageInstance::instantiate() const {
  if (!isType<T>())
    return boost::shared_ptr<T>();

  boost::shared_ptr<T> ptr = new T();
  return deserialize(message_, ptr) ? ptr : boost::shared_ptr<T>();
}

template<typename Stream>
void MessageInstance::write(Stream& stream) const {
  memcpy(stream.advance(size()), buffer_cast<const void *>(message_), size());
}

template<typename Stream>
void MessageInstance::read(Stream& stream)
{
  stream.getLength();
  stream.getData();
  copy(ConstBuffer(stream.getData(), stream.getLength()));
}

} // namespace rosserial
} // namespace hector_serialization


namespace ros {
namespace message_traits {

template<>
struct MD5Sum<hector_serialization::rosserial::MessageInstance>
{
  static const char* value(const hector_serialization::rosserial::MessageInstance& m) { return m.getMD5Sum().c_str(); }
  static const char* value() { return "*"; }
};

template<>
struct DataType<hector_serialization::rosserial::MessageInstance>
{
   static const char* value(const hector_serialization::rosserial::MessageInstance& m) { return m.getDataType().c_str(); }
   static const char* value() { return "*"; }
};

template<>
struct Definition<hector_serialization::rosserial::MessageInstance>
{
   static const char* value(const hector_serialization::rosserial::MessageInstance& m) { return m.getMessageDefinition().c_str(); }
   static const char* value() { return ""; }
};

} // namespace message_traits


namespace service_traits {

template<>
struct MD5Sum<hector_serialization::rosserial::MessageInstance>
{
  static const char* value(const hector_serialization::rosserial::MessageInstance& m) { return m.getMD5Sum().c_str(); }
  static const char* value() { return "*"; }
};

template<>
struct DataType<hector_serialization::rosserial::MessageInstance>
{
   static const char* value(const hector_serialization::rosserial::MessageInstance& m) { return m.getDataType().c_str(); }
   static const char* value() { return "*"; }
};

} // namespace message_traits

namespace serialization
{

template<>
struct Serializer<hector_serialization::rosserial::MessageInstance>
{
  template<typename Stream>
  inline static void write(Stream& stream, const hector_serialization::rosserial::MessageInstance& m) {
    m.write(stream);
  }

  inline static uint32_t serializedLength(const hector_serialization::rosserial::MessageInstance& m) {
    return m.size();
  }

  template<typename Stream>
  inline static void read(Stream& stream, hector_serialization::rosserial::MessageInstance& m)
  {
    m.read(stream);
  }
};

template<>
struct PreDeserialize<hector_serialization::rosserial::MessageInstance>
{
  static void notify(const PreDeserializeParams<hector_serialization::rosserial::MessageInstance>& params)
  {
    std::string md5      = (*params.connection_header)["md5sum"];
    std::string datatype = (*params.connection_header)["type"];
    std::string msg_def  = (*params.connection_header)["message_definition"];
    std::string latching  = (*params.connection_header)["latching"];

//    typedef std::map<std::string, std::string> map_t;
//    boost::shared_ptr<map_t> shmap(new map_t(*params.connection_header));
//    params.message->__connection_header = shmap;
    params.message->morph(md5, datatype, msg_def, latching);
  }
};

} // namespace serialization
} // namespace ros

#endif // HECTOR_SERIALIZATION_ROSSERIAL_MESSAGE_INSTANCE_H
