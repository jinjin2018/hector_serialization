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
#include <boost/bind.hpp>

namespace hector_serialization {

  ChannelElement::ChannelElement()
    : next_(0)
  {}

  ChannelElement::~ChannelElement()
  {}

  ChannelElement* ChannelElement::setNext(ChannelElement* next)
  {
    if (next_) {
      this->removeOutput(next_);
      next_->removeCallback(this);
    }
    next_ = next;
    if (next_) {
      next_->addCallback(this);
      this->addOutput(next_);
    }
    return next_;
  }

  bool ChannelElement::write(const BufferSequence &sequence, const Context& context) {
    if (writers_.empty()) return false;

    bool result = true;
    std::list<WriteHandler*>::const_iterator it = writers_.begin();
    for( ; it != writers_.end(); ++it) result = (*it)->write(sequence, context) && result;

    return result;
  }

  bool ChannelElement::read(const MutableBuffer& buffer)
  {
    std::size_t n = boost::asio::buffer_size(buffer);

    if (stream()) {
      if (stream()->size() >= n) {
        std::copy(boost::asio::buffers_begin(stream()->data()), boost::asio::buffers_begin(stream()->data()) + n, boost::asio::buffer_cast<char *>(buffer));
        stream()->consume(n);
        return true;
      }
    } else if (size() >= n) {
      memcpy(boost::asio::buffer_cast<void *>(buffer), boost::asio::buffer_cast<const char *>(data()), n);
      return true;
    }

    return false;
  }

  bool ChannelElement::readsome(const MutableBuffer& buffer, std::size_t& n)
  {
    n = boost::asio::buffer_size(buffer);

    if (stream())
      n = std::min(n, stream()->size());
    else
      n = std::min(n, size());

    return read(MutableBuffer(boost::asio::buffer_cast<void*>(buffer), n));
  }

  bool ChannelElement::wait(double timeout)
  {
    return wait(boost::posix_time::seconds((int)timeout) + boost::posix_time::milliseconds((timeout - (int)timeout) * 1000.0));
  }

  bool ChannelElement::wait(const boost::posix_time::time_duration& timeout)
  {
    if (!next_) return false;
    return next_->wait(timeout);
  }

  StreamBuf *ChannelElement::stream()
  {
    return 0;
  }

  ConstBuffer ChannelElement::data()
  {
    if (stream()) return stream()->data();
    if (!next_) return EmptyBuffer();
    return next_->data();
  }

  std::size_t ChannelElement::size() {
    return stream() ? stream()->size() : boost::asio::buffer_size(data());
  }

  void ChannelElement::consume(std::size_t n) {
    if (!stream()) return;
    stream()->consume(n);
  }

  ChannelElement& ChannelElement::addCallback(ReadHandler* handler)
  {
    readers_.push_back(handler);
    return *this;
  }

  void ChannelElement::removeCallback(ReadHandler* handler)
  {
    std::list<ReadHandler*>::iterator it = std::find(readers_.begin(), readers_.end(), handler);
    if (it != readers_.end()) readers_.erase(it);
  }

  ChannelElement& ChannelElement::addOutput(WriteHandler* handler)
  {
    writers_.push_back(handler);
    return *this;
  }

  void ChannelElement::removeOutput(WriteHandler* handler)
  {
    std::list<WriteHandler*>::iterator it = std::find(writers_.begin(), writers_.end(), handler);
    if (it != writers_.end()) writers_.erase(it);
  }

//  void ChannelElement::handle(ChannelElement* channel, const Context& context)
//  {
//    trigger(context);
//  }

//  void ChannelElement::trigger(const Context& context)
//  {
//    std::list<ReadHandler*>::const_iterator it = readers_.begin();
//    for( ; it != readers_.end(); ++it) (*it)->handle(this, context);
//  }

  void ChannelElement::handle(const ConstBuffers1& buffer, const Context& context)
  {
    std::list<ReadHandler*>::const_iterator it = readers_.begin();
    for( ; it != readers_.end(); ++it) (*it)->handle(buffer, context);
  }

} // namespace hector_serialization
