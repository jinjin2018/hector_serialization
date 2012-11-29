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

#ifndef HECTOR_SERIALIZATION_SERIALIZATION_H
#define HECTOR_SERIALIZATION_SERIALIZATION_H

#include <hector_serialization/types.h>
#include <hector_serialization/buffer.h>
#include <hector_serialization/channel.h>
#include <ros/serialization.h>

namespace hector_serialization {

  class OStream : public ros::serialization::OStream {
  public:
    OStream(const MutableBuffer& buffer)
      : ros::serialization::OStream(boost::asio::buffer_cast<uint8_t*>(buffer), boost::asio::buffer_size(buffer))
      , begin_(getData())
    {}

    std::size_t getSize() {
      return (getData() - begin_);
    }

    uint8_t* begin_;
  };

  class IStream : public ros::serialization::IStream {
  public:
    IStream(const ConstBuffer &buffer)
      : ros::serialization::IStream(const_cast<uint8_t*>(boost::asio::buffer_cast<const uint8_t*>(buffer)), boost::asio::buffer_size(buffer))
      , begin_(getData())
    {}

    std::size_t getSize() {
      return (getData() - begin_);
    }

    uint8_t* begin_;
  };

  /**
   * \brief Serialize an object. Default serialization simply forwards to ros::serialization.
   */
  template<typename T>
  inline std::size_t serialize(const MutableBuffer& buffer, const T& t)
  {
    OStream stream(buffer);
    ros::serialization::serialize(stream, t);
    return stream.getSize();
  }

  /**
   * \brief Serialize an object into a new SharedBuffer.
   */
  template<typename T>
  inline SharedBuffer serialize(const T& t)
  {
    SharedBuffer buffer(serializationLength(t));
    serialize(buffer, t);
    return buffer;
  }

  /**
   * \brief Serialize an object without throwing an exception
   */
  template<typename Target, typename T>
  inline bool try_serialize(Target target, const T& t)
  {
    try {
      serialize(target, t);
    } catch(...) {
      return false;
    }
    return true;
  }

  /**
   * \brief Deserialize an object. Default serialization simply forwards to ros::serialization.
   */
  template<typename T>
  inline std::size_t deserialize(const ConstBuffer& buffer, T& t)
  {
    IStream stream(buffer);
    ros::serialization::deserialize(stream, t);
    return stream.getSize();
  }

  /**
   * \brief Deserialize an object from a channel.
   */
  template<typename T>
  inline std::size_t deserialize(ChannelElement& channel, T& t)
  {
    std::size_t n = deserialize(channel.data(), t);
    channel.consume(n);
    return n;
  }

  /**
   * \brief Deserialize an object without throwing an exception
   */
  template<typename Source, typename T>
  inline bool try_deserialize(Source source, T& t)
  {
    try {
      deserialize(source, t);
    } catch(...) {
      return false;
    }
    return true;
  }

  /**
   * \brief Deserialize an object from a channel.
   */
  template<typename T>
  inline bool deserialize(ChannelElement& channel, T& t)
  {
    std::size_t n = deserialize(channel.data(), t);
    channel.consume(n);
    return n;
  }

  /**
   * \brief Determine the serialized length of an object. Default serialization simply forwards to ros::serialization.
   */
  template<typename T>
  inline std::size_t serializationLength(const T& t)
  {
    return ros::serialization::serializationLength(t);
  }

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_SERIALIZATION_H
