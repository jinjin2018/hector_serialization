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

#ifndef HECTOR_SERIALIZATION_PROTOCOL_H
#define HECTOR_SERIALIZATION_PROTOCOL_H

#include <hector_serialization/channel.h>
#include <boost/function.hpp>

namespace hector_serialization {

struct Protocol
{
  virtual ~Protocol() {}
};

template <class Derived>
struct ProtocolInfo
{
  class Address;
  class SendHeader;
  class ReceiveHeader;
  class Status;
  class ContextElement;
};

template <class Derived, typename ProtocolInfo = ProtocolInfo<Derived> >
class Protocol_ : public Protocol
{
public:
  typedef typename ProtocolInfo::Address Address;
  typedef typename ProtocolInfo::SendHeader SendHeader;
  typedef typename ProtocolInfo::ReceiveHeader ReceiveHeader;
  typedef typename ProtocolInfo::Status Status;
  typedef typename ProtocolInfo::ContextElement ContextElement;

  virtual ~Protocol_() {}

  /* get/set header info */
  Protocol_<Derived,ProtocolInfo>& header(const SendHeader& header)
  {
    send_header_ = header;
    return *this;
  }
  const ReceiveHeader& header() const { return receive_header_; }

  /* callbacks with header information */
  typedef boost::function<void(const ConstBuffer&, const ReceiveHeader&)> ReceiveCallback;
  typedef boost::function<void(const Status&)> StatusCallback;

  void addCallback(const ReceiveCallback& callback)
  {
    callbacks_.push_back(callback);
  }

  void addCallback(const ReceiveCallback& callback, const ReceiveHeader& header)
  {
    filtered_callbacks_.push_back(std::make_pair<ReceiveHeader,ReceiveCallback>(header, callback));
  }

  void addCallback(const StatusCallback& callback)
  {
    status_callbacks_.push_back(callback);
  }

  void removeCallback(const ReceiveCallback& callback) {
    for(typename ReceiveCallbacks::iterator it = callbacks_.begin(); it != callbacks_.end(); ) {
      if (*it == callback) {
        it = callbacks_.erase(it);
        continue;
      }
      ++it;
    }

    for(typename FilteredCallbacks::iterator it = filtered_callbacks_.begin(); it != filtered_callbacks_.end(); ) {
      if (it->second == callback) {
        it = filtered_callbacks_.erase(it);
        continue;
      }
      ++it;
    }
  }

  void removeCallback(const StatusCallback& callback) {
    status_callbacks_.erase(std::find(status_callbacks_.begin(), status_callbacks_.end(), callback));
  }

protected:
  virtual void receiveCallback(const ConstBuffer &buffer, const ReceiveHeader& header)
  {
    for(typename ReceiveCallbacks::iterator it = callbacks_.begin(); it != callbacks_.end(); ++it) (*it)(buffer, header);
    for(typename FilteredCallbacks::iterator it = filtered_callbacks_.begin(); it != filtered_callbacks_.end(); ++it) if (header == it->first) it->second(buffer, header);
  }

  virtual void statusCallback(const Status& header)
  {
    for(typename StatusCallbacks::iterator it = status_callbacks_.begin(); it != status_callbacks_.end(); ++it) (*it)(header);
  }

  SendHeader send_header_;
  ReceiveHeader receive_header_;

private:
  typedef std::list<ReceiveCallback> ReceiveCallbacks;
  ReceiveCallbacks callbacks_;
  typedef std::list<std::pair<ReceiveHeader,ReceiveCallback> > FilteredCallbacks;
  FilteredCallbacks filtered_callbacks_;
  typedef std::list<StatusCallback> StatusCallbacks;
  StatusCallbacks status_callbacks_;
};

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_PROTOCOL_H
