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

#include <hector_serialization/channel.h>

#ifndef HECTOR_SERIALIZATION_BUFFERED_CHANNEL_H
#define HECTOR_SERIALIZATION_BUFFERED_CHANNEL_H

namespace hector_serialization {

class BufferedChannelElement : public ChannelElement
{
public:
  BufferedChannelElement(const std::size_t buffer_size = 8192)
  {}
  virtual ~BufferedChannelElement() {}

  virtual StreamBuf *stream() {
    return &in_;
  }

  virtual void handle(const ConstBuffer &buffer, const Context &context)
  {
    boost::mutex::scoped_lock lock(mutex_);
    boost::asio::const_buffers_1 source(buffer);
    MutableBuffer temp = in_.prepare(buffer_size(buffer));
    std::copy(buffers_begin(source), buffers_end(source), buffers_begin(temp));
    in_.commit(buffer_size(buffer));
    handle(&in_, context);
    update_condition_.notify_all();
  }

  virtual void handle(StreamBuf *stream, const Context &context) = 0;

  virtual bool wait(const boost::posix_time::time_duration &timeout)
  {
    boost::mutex::scoped_lock lock(mutex_);
    return update_condition_.timed_wait(lock, timeout);
  }

  boost::mutex& mutex() { return mutex_; }

private:
  StreamBuf in_;
  boost::mutex mutex_;
  boost::condition update_condition_;
};

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_BUFFERED_CHANNEL_H
