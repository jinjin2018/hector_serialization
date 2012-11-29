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

#ifndef HECTOR_SERIALIZATION_PROTOCOL_XBEE_H
#define HECTOR_SERIALIZATION_PROTOCOL_XBEE_H

#include <hector_serialization/buffered_channel.h>
#include <hector_serialization/protocol.h>
#include <stdexcept>

namespace hector_serialization {

class XBee;

template <>
struct ProtocolInfo<XBee> {
  struct Address : public Context::Element<Address> {
    uint64_t address;

    Address() : address(0), bit64_(false) {}
    Address(uint16_t address) { set16Bit(address); }
    Address(uint64_t address) { set64Bit(address); }

    Address& set16Bit(uint16_t address = 0) { this->address = address; bit64_ = false; return *this; }
    Address& set64Bit(uint64_t address = 0) { this->address = address; bit64_ = true; return *this; }

    bool is16Bit() const { return !bit64_; }
    bool is64Bit() const { return bit64_; }

    std::string toString() const { return boost::lexical_cast<std::string>(address); }

    bool isEqual(const Address& other) const {
      return this->bit64_ == other.bit64_ && this->address == other.address;
    }

  private:
    bool bit64_;
  };

  struct SendHeader : public Context::Element<SendHeader> {
    uint8_t frame_id;
    Address destination_address;
    uint8_t options;
    enum { DISABLE_ACK = 0x01, PAN_BROADCAST = 0x04 };

    SendHeader() : frame_id(), destination_address(), options() {}
    SendHeader& setFrameId(uint8_t frame_id) { this->frame_id = frame_id; return *this; }
    SendHeader& setDestinationAddress(const Address& destination_address) { this->destination_address = destination_address; return *this; }
    SendHeader& setOptions(uint8_t options) { this->options = options; return *this; }

    bool isEqual(const SendHeader& other) const {
      return this->destination_address == other.destination_address;
    }

    std::string identifier() const { return destination_address.identifier(); }
  };

  struct ReceiveHeader : public Context::Element<ReceiveHeader> {
    uint8_t frame_id;
    Address source_address;
    uint8_t rssi;
    uint8_t options;
    enum { ADDRESS_BROADCAST = 0x01, PAN_BROADCAST = 0x02 };

    ReceiveHeader() : frame_id(), source_address(), rssi(), options() {}
    ReceiveHeader& setRSSI(uint8_t rssi) { this->rssi = rssi; return *this; }
    ReceiveHeader& setFrameId(uint8_t frame_id) { this->frame_id = frame_id; return *this; }
    ReceiveHeader& setSourceAddress(const Address& source_address) { this->source_address = source_address; return *this; }
    ReceiveHeader& setOptions(uint8_t options) { this->options = options; return *this; }

    bool isEuqal(const ReceiveHeader& other) const {
      return this->source_address == other.source_address;
    }

    std::string identifier() const { return source_address.identifier(); }
  };

  struct Status {
    uint8_t frame_id;
    uint8_t status;
    enum { SUCCESS = 0x00, NO_ACK = 0x01, CCA_FAILURE = 0x02, PURGED = 0x03 };

    Status() : frame_id(0), status(0) {}
    operator void*() { return reinterpret_cast<void*>(status == SUCCESS); }
    operator uint8_t&() { return status; }
  };
};

class XBee : public Protocol_<XBee>, public BufferedChannelElement {
public:
  struct ProtocolError : public std::runtime_error
  {
    ProtocolError(const std::string& str) : std::runtime_error(str) {}
  };

  struct Checksum {
    Checksum(const ConstBuffer& buffer);
    Checksum(BufferIterator begin, BufferIterator end);
    Checksum(uint8_t checksum) : checksum_(checksum) {}
    const uint8_t& checksum() const { return checksum_; }
    operator uint8_t&() { return checksum_; }

  private:
    uint8_t checksum_;
    template <typename BufferSequence> void calculate_checksum(boost::asio::buffers_iterator<BufferSequence> begin, boost::asio::buffers_iterator<BufferSequence> end);
  };

  struct ATCommand {
    uint8_t frame_id;
    boost::array<uint8_t,2> command;

    ATCommand(const char *command = 0) : frame_id() { *this = command; }
    ATCommand& operator=(const char *command) { if (!command) return *this; std::copy(command, command + 2, this->command.data()); return *this; }

    std::vector<uint8_t> value;
    template <typename T> ATCommand& setValue(const T& val);
  };

  struct ATResponse {
    uint8_t frame_id;
    boost::array<uint8_t,2> command;
    uint8_t status;
    enum { OK = 0x00, ERROR = 0x01 };
    operator void*() { return reinterpret_cast<void*>(status == OK); }

    ATResponse() : frame_id(), command(), status(ERROR) {}

    std::vector<uint8_t> value;
    template <typename T> T as() const;
  };

  XBee(ChannelElement* next = 0);
  virtual ~XBee();

  bool configure();
  bool configure(ATCommand& command, ATResponse& response);
  bool isConfigured();

  XBee& setTimeout(const boost::posix_time::time_duration& timeout) { timeout_ = timeout; return *this; }
  const boost::posix_time::time_duration& getTimeout() const { return timeout_; }

  static SendHeader broadcast();
  using ChannelElement::write;

protected:
  bool write(const BufferSequence &sequence);
  // void handle(ChannelElement* channel);
  void handle(StreamBuf *stream, const Context &context = Context());
  void handleAPI(const ConstBuffer& buffer, const Context& context = Context());

  Status status_;

  ATResponse response_;
  boost::mutex response_mutex_;
  boost::condition response_condition_;

private:
  bool configured_;
  boost::posix_time::time_duration timeout_;
  uint8_t auto_frame_id_;
};

template <typename T> T XBee::ATResponse::as() const { T val(0); for(std::vector<uint8_t>::const_iterator it = value.begin(); it < value.end(); ++it) val = val << 8 | *it; return val; }
template <> std::string XBee::ATResponse::as<std::string>() const { return std::string(reinterpret_cast<const char *>(value.data()), value.size()); }
template <typename T> XBee::ATCommand& XBee::ATCommand::setValue(const T& val) { value.resize(sizeof(val)); for(std::size_t i = 0; i < sizeof(val); ++i) value[sizeof(val) - i - 1] = (val << (8*i)) & 0xFF; return *this; }
template <> XBee::ATCommand& XBee::ATCommand::setValue<std::string>(const std::string& str) { value.resize(str.length()); std::copy(str.begin(), str.end(), value.begin()); return *this; }

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_PROTOCOL_XBEE_H
