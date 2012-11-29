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

#ifndef HECTOR_SERIALIZATION_CHANNEL_H
#define HECTOR_SERIALIZATION_CHANNEL_H

#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <list>

#include <hector_serialization/types.h>
#include <hector_serialization/buffer.h>
#include <hector_serialization/context.h>

namespace hector_serialization {

  class ReadHandler {
  public:
    // virtual void handle(ChannelElement* channel, const Context& context = Context()) = 0;
    virtual void handle(const ConstBuffer& buffer, const Context& context = Context()) = 0;
  };

  class WriteHandler {
  public:
    virtual bool write(const BufferSequence &sequence, const Context& context) = 0;
  };

  class ChannelElement : public ReadHandler, public WriteHandler
  {
  public:
    ChannelElement();
    virtual ~ChannelElement();

    /* get pointers to child ChannelElements in the chain */
    ChannelElement* next() const { return next_; }
    template <typename T> T* layer(int layer = -1) const;

    /* set next channel in the chain */
    ChannelElement* setNext(ChannelElement* next);

    /* configuration */
    virtual bool configure() { return next_ ? next_->configure() : true; }
    virtual bool isConfigured() { return next_ ? next_->isConfigured() : true; }
    virtual void cleanup() { if (next_) next_->cleanup(); }

    /* write data to the channel */
    bool write(const ConstBuffer& buffer, const Context& context = Context()) { return write(BufferSequence(1, buffer), context); }
    bool write(const void *data, std::size_t size, const Context& context = Context()) { return write(ConstBuffer(data, size), context); }
    virtual bool write(const BufferSequence &sequence, const Context& context = Context());

    /* read data from to the channel */
    bool read(void *data, std::size_t n) { return read(MutableBuffer(data, n)); }
    bool read(const MutableBuffer& buffer);
    bool readsome(void *data, std::size_t& n) { return readsome(MutableBuffer(data, n), n); }
    virtual bool readsome(const MutableBuffer& buffer, std::size_t& n);
    bool wait(double timeout);
    virtual bool wait(const boost::posix_time::time_duration& timeout);

    /* direct access to the underlying StreamBuf (if available) or ConstBuffer */
    virtual StreamBuf *stream();
    virtual ConstBuffer data();
    virtual std::size_t size();
    virtual void consume(std::size_t n);

    /* add and remove receive handlers */
    ChannelElement& addCallback(ReadHandler* handler);
    void removeCallback(ReadHandler* handler);

    /* add and remove writers */
    ChannelElement& addOutput(WriteHandler* handler);
    void removeOutput(WriteHandler* handler);

  protected:
//    virtual void handle(ChannelElement* channel, const Context& context = Context());
//    virtual void trigger(const Context& context = Context());
    virtual void handle(const ConstBuffer& buffer, const Context& context = Context());

  private:
    ChannelElement* next_;
    std::list<WriteHandler*> writers_;
    std::list<ReadHandler*> readers_;
  };

  template <typename T>
  T* ChannelElement::layer(int layer) const
  {
    ChannelElement* p = next();
    while(p) {
      if (layer <= 0) {
        T* target = dynamic_cast<T*>(p);
        if (target) return target;
      }
      if (layer == 0) return 0;
      if (layer > 0) layer--;
      p = p->next();
    }
    return 0;
  }

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_CHANNEL_H
