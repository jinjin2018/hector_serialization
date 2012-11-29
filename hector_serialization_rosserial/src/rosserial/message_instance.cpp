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

#include <hector_serialization/rosserial/message_instance.h>

using ros::Time;
using boost::shared_ptr;

namespace hector_serialization {
namespace rosserial {

MessageInstance::MessageInstance()
  : message_(EmptyBuffer())
  , buffer_(0)
  , buffer_alloc_(0)
{
}

MessageInstance::MessageInstance(const MessageInstance &other)
  : message_(other.message_)
  , topic_info_(other.topic_info_)
  , message_definition_(other.message_definition_)
  , latching_(other.latching_)
  , buffer_(0)
  , buffer_alloc_(0)
{
  if (other.buffer_) copy(other.getMessage());
}

MessageInstance::MessageInstance(const TopicInfo &topic_info)
  : message_(EmptyBuffer())
  , topic_info_(topic_info)
  , buffer_(0)
  , buffer_alloc_(0)
{
}

MessageInstance::MessageInstance(const ConstBuffer &message, const TopicInfo &topic_info)
  : message_(message)
  , topic_info_(topic_info)
  , buffer_(0)
  , buffer_alloc_(0)
{
}

MessageInstance::~MessageInstance()
{
  if (buffer_) delete[] buffer_;
}

MessageInstance &MessageInstance::operator=(const ConstBuffer &message)
{
  this->message_ = message;
  return *this;
}

std::string const& MessageInstance::getTopic()               const { return topic_info_.topic_name; }
std::string const& MessageInstance::getDataType()            const { return topic_info_.message_type; }
std::string const& MessageInstance::getMD5Sum()              const { return topic_info_.md5sum;   }
std::string const& MessageInstance::getMessageDefinition()   const { return message_definition_; }

ConstBuffer const& MessageInstance::getMessage() const { return message_; }

void MessageInstance::morph(const std::string& md5sum, const std::string& datatype, const std::string& msg_def,
           const std::string& latching)
{
  topic_info_.md5sum = md5sum;
  topic_info_.message_type = datatype;
  message_definition_ = msg_def;

}

uint32_t MessageInstance::size() const {
  return buffer_size(message_);
}

const uint8_t *MessageInstance::data() const {
  return buffer_cast<const uint8_t *>(message_);
}

void MessageInstance::copy(const ConstBuffer &source)
{
  std::size_t size = buffer_size(source);

  if (size > buffer_alloc_) {
    delete[] buffer_;
    buffer_ = new uint8_t[size];
    buffer_alloc_ = size;
  }

  memcpy(buffer_, buffer_cast<const void *>(source), size);
  message_ = ConstBuffer(buffer_, size);
}

} // namespace rosserial
} // namespace hector_serialization
