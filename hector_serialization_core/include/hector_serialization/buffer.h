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

#ifndef HECTOR_SERIALIZATION_BUFFER_H
#define HECTOR_SERIALIZATION_BUFFER_H

#include <boost/shared_ptr.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/asio/buffers_iterator.hpp>
#include <list>

namespace hector_serialization {

  typedef boost::asio::const_buffer ConstBuffer;
  typedef boost::asio::const_buffers_1 ConstBuffers1;
  typedef boost::asio::mutable_buffer MutableBuffer;
  typedef boost::asio::mutable_buffers_1 MutableBuffers1;
  typedef std::list<boost::asio::const_buffer> BufferSequence;

  typedef boost::asio::streambuf StreamBuf;

  using boost::asio::buffer_cast;
  using boost::asio::buffer_size;
  using boost::asio::buffers_begin;
  using boost::asio::buffers_end;
  using boost::asio::buffers_iterator;

  namespace detail {
    template <typename T>
    class SharedBufferHelper_ {
    public:
      template <typename OtherT> SharedBufferHelper_(const OtherT& other) : buffer_(new std::vector<T>(other)) {}
      SharedBufferHelper_(const std::size_t& size) : buffer_(new std::vector<T>(size)) {}
      SharedBufferHelper_(const SharedBufferHelper_<T>& other) : buffer_(other.buffer_) {}

    protected:
      typename boost::shared_ptr<std::vector<T> > buffer_;
    };

    template <typename T>
    class SharedBuffer_ : public SharedBufferHelper_<T>, public MutableBuffers1
    {
    public:
      template <typename Arg1>
      SharedBuffer_(const Arg1& arg)
        : SharedBufferHelper_<T>(arg)
        , MutableBuffers1(this->buffer_->data(), this->buffer_->size())
      {}

      operator ConstBuffers1() { return ConstBuffers1(this->buffer_->data(), this->buffer_->size()); }
    };

    template <typename T>
    class SharedConstBuffer_ : public SharedBufferHelper_<T>, public ConstBuffers1
    {
    public:
      template <typename Arg1>
      SharedConstBuffer_(const Arg1& arg)
        : SharedBufferHelper_<T>(arg)
        , ConstBuffers1(this->buffer_->data(), this->buffer_->size())
      {}
    };
  }

  typedef detail::SharedBuffer_<unsigned char> SharedBuffer;
  typedef detail::SharedConstBuffer_<unsigned char> SharedConstBuffer;

  template <typename ConstBufferSequence, typename MutableBufferSequence>
  static inline std::size_t copy(const ConstBufferSequence& source, const MutableBufferSequence& destination) {
    boost::asio::buffers_iterator<ConstBufferSequence> source_it        = boost::asio::buffers_begin<ConstBufferSequence>(source);
    boost::asio::buffers_iterator<MutableBufferSequence> destination_it = buffers_begin<MutableBufferSequence>(destination);
    for(; source_it != boost::asio::buffers_end<ConstBufferSequence>(source) && destination_it != boost::asio::buffers_end<MutableBufferSequence>(destination); ++source_it, ++destination_it)
      *destination_it = *source_it;
    return source_it - boost::asio::buffers_begin<ConstBufferSequence>(source);
  }

  class EmptyBuffer : public ConstBuffers1
  {
  public:
    EmptyBuffer() : ConstBuffers1(0, 0) {}
  };

  static inline BufferSequence operator+(const BufferSequence& sequence1, const BufferSequence& sequence2) {
    BufferSequence result;
    result.insert(result.end(), sequence1.begin(), sequence1.end());
    result.insert(result.end(), sequence2.begin(), sequence2.end());
    return result;
  }

  static inline BufferSequence operator+(const BufferSequence& sequence1, const ConstBuffers1& buffer2) {
    BufferSequence result;
    result.insert(result.end(), sequence1.begin(), sequence1.end());
    result.push_back(buffer2);
    return result;
  }

  static inline BufferSequence operator+(const ConstBuffers1& buffer1, const ConstBuffers1& buffer2) {
    BufferSequence result(2);
    result.push_back(buffer1);
    result.push_back(buffer2);
    return result;
  }

} // namespace hector_serialization

namespace ros { namespace serialization {

template<typename Stream>
void serialize(Stream& stream, const hector_serialization::ConstBuffers1& buffer)
{
  std::size_t n = hector_serialization::buffer_size(buffer);
  memcpy(stream.advance(n), hector_serialization::buffer_cast<const void*>(buffer), n);
}

template<typename Stream>
void deserialize(Stream& stream, const hector_serialization::MutableBuffers1& buffer)
{
  std::size_t n = std::min(std::size_t(stream.getLength()), hector_serialization::buffer_size(buffer));
  memcpy(hector_serialization::buffer_cast<const void*>(buffer), stream.advance(n), n);
}

}} // namespace ros::serialiation

#endif // HECTOR_SERIALIZATION_BUFFER_H
