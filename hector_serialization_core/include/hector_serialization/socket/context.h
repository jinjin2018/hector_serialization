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

#ifndef HECTOR_SERIALIZATION_SOCKET_CONTEXT_H
#define HECTOR_SERIALIZATION_SOCKET_CONTEXT_H

#include <hector_serialization/context.h>
#include <boost/asio/ip/address.hpp>

namespace hector_serialization {
namespace socket {

struct ContextElementParameters {
  static std::string prefix;
  static unsigned int host_bits;

  template <typename bytes_type>
  static unsigned long generateNumericId(const bytes_type& bytes)
  {
    unsigned long id = 0;
    unsigned int bits = 0;
    typename bytes_type::const_reverse_iterator byte = bytes.rbegin();

    while(bits < host_bits && byte < bytes.rend()) {
      if (host_bits - bits >= 8) {
        id = id | (*byte++ << bits);
        bits += 8;
      } else {
        typename bytes_type::value_type mask = (1u << (host_bits - bits)) - 1;
        id = id | ((*byte++ & mask) << bits);
        bits = host_bits;
      }
    }

    return id;
  }
};

template <typename EndpointT>
class EndpointContextElement : public hector_serialization::Context::Element<EndpointContextElement<EndpointT> >
{
private:
  EndpointT endpoint_;

public:
  EndpointContextElement() {}
  EndpointContextElement(const EndpointT& endpoint) : endpoint_(endpoint) {}
  virtual ~EndpointContextElement() {}

  virtual bool isEqual(const EndpointContextElement& other) const {
    return (other.endpoint_ == this->endpoint_);
  }

  virtual std::string identifier() const {
    if (endpoint_.address().is_v4()) {
      return ContextElementParameters::prefix + boost::lexical_cast<std::string>(ContextElementParameters::generateNumericId(endpoint_.address().to_v4().to_bytes()));
    } else if (endpoint_.address().is_v6()) {
      return ContextElementParameters::prefix + boost::lexical_cast<std::string>(ContextElementParameters::generateNumericId(endpoint_.address().to_v6().to_bytes()));
    }
    return boost::lexical_cast<std::string>(endpoint_);
  }

  EndpointT &endpoint() { return endpoint_; }
  const EndpointT &endpoint() const { return endpoint_; }
};

} // namespace hector_serialization
} // namespace socket

#endif // HECTOR_SERIALIZATION_SOCKET_CONTEXT_H
