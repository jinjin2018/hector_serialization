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

#include <hector_serialization/xbee.h>
#include <hector_serialization/serialization.h>

namespace hector_serialization {

template <> std::size_t serialize(const MutableBuffer &buffer, const XBee::SendHeader &header);
template <> std::size_t serializationLength(const typename XBee::SendHeader& header);
template <> std::size_t serialize(const MutableBuffer &buffer, const XBee::ATCommand &command);
template <> std::size_t serializationLength(const typename XBee::ATCommand& command);
template <> std::size_t deserialize(const ConstBuffer& buffer, typename XBee::ReceiveHeader& header);
template <> std::size_t deserialize(const ConstBuffer& buffer, typename XBee::Status& status);
template <> std::size_t deserialize(const ConstBuffer& buffer, typename XBee::ATResponse& response);

XBee::XBee(ChannelElement* next)
  : configured_(false)
  , timeout_(0,0,1)
  , auto_frame_id_(0)
{
  this->setNext(next);
}

XBee::~XBee()
{

}

bool XBee::configure()
{
  ATCommand command;
  ATResponse response;

  // query MY
  command = ATCommand("MY");
  if (!configure(command, response) || !response) {
    std::cout << "ATCommand timed out" << std::endl;
    return false;
  }
  std::cout << "MY is " << response.as<uint16_t>() << std::endl;

  return true;
}

bool XBee::configure(ATCommand& command, ATResponse& response)
{
  boost::mutex::scoped_lock lock(response_mutex_);

  BufferSequence sequence;
  uint16_t length = serializationLength(command) + 1;

  uint8_t api[4];
  api[0] = 0x7e;
  api[1] = length >> 8;
  api[2] = length & 0xFF;
  api[3] = 0x08;
  sequence.push_back(ConstBuffer(&api, sizeof(api)));

  if (command.frame_id == 0) command.frame_id = ++auto_frame_id_;

  uint8_t serialized_command[serializationLength(command)];
  serialize(MutableBuffer(serialized_command, sizeof(serialized_command)), command);
  sequence.push_back(ConstBuffer(serialized_command, sizeof(serialized_command)));

  Checksum checksum = Checksum(boost::asio::buffers_begin(sequence) + 3, boost::asio::buffers_end(sequence));
  sequence.push_back(ConstBuffer(&checksum.checksum(), 1));

  if (!ChannelElement::write(sequence)) return false;

  if (!response_condition_.timed_wait(lock, timeout_)) return false;
  response = response_;
  return true;
}

bool XBee::isConfigured()
{
  return configured_;
}

XBee::SendHeader XBee::broadcast()
{
  SendHeader header;
  header.destination_address.set16Bit(0xFFFF);
  return header;
}

bool XBee::write(const BufferSequence &payload)
{
  BufferSequence sequence;
  uint16_t length = serializationLength(send_header_) + 1 + (boost::asio::buffers_end(payload) - boost::asio::buffers_begin(payload));

  uint8_t api[4];
  api[0] = 0x7e;
  api[1] = length >> 8;
  api[2] = length & 0xFF;
  api[3] = send_header_.destination_address.is64Bit() ? 0x00 : 0x01;
  sequence.push_back(ConstBuffer(&api, sizeof(api)));

  if (send_header_.frame_id == 0) send_header_.frame_id = ++auto_frame_id_;

  uint8_t serialized_header[serializationLength(send_header_)];
  serialize(MutableBuffer(serialized_header, sizeof(serialized_header)), send_header_);
  sequence.push_back(ConstBuffer(serialized_header, sizeof(serialized_header)));

  sequence.insert(sequence.end(), payload.begin(), payload.end());

  Checksum checksum = Checksum(boost::asio::buffers_begin(sequence) + 3, boost::asio::buffers_end(sequence));
  sequence.push_back(ConstBuffer(&checksum.checksum(), 1));

  return ChannelElement::write(sequence);
}

void XBee::handle(StreamBuf *in, const Context &context)
{
  IStream stream(in->data());
  while(stream.getLength() > 0 && *(stream.getData()) != 0x7e) { stream.advance(1); }

  uint8_t *packet = stream.getData();
  if (stream.getLength() >= 4 && packet[0] == 0x7e) {
    uint16_t length = packet[1] << 8 | packet[2];
    if (stream.getLength() >= uint32_t(length + 4)) {
      ConstBuffers1 payload(&packet[3], length);
      if (Checksum(payload) == packet[3 + length]) {
        handleAPI(payload, context);
        stream.advance(length + 4);
      }
    }
  }
  in->consume(stream.getSize());
}

void XBee::handleAPI(const ConstBuffers1& buffer, const Context& context)
{
  const uint8_t *api = buffer_cast<const uint8_t*>(buffer);

  switch(api[0]) {
    case 0x80:
    case 0x81:
      {
        if (api[0] == 0x80) receive_header_.source_address.set16Bit(); else receive_header_.source_address.set64Bit();
        ConstBuffers1 payload_(buffer + 1 + deserialize(buffer + 1, receive_header_));
        ChannelElement::handle(payload_, context + receive_header_);
        receiveCallback(payload_, receive_header_);
        break;
      }

    case 0x88:
      {
        boost::mutex::scoped_lock lock(response_mutex_);
        deserialize(buffer + 1, response_);
        response_condition_.notify_all();
      }
      break;

    case 0x89:
      deserialize(buffer + 1, status_);
      statusCallback(status_);
      break;

    default:
      break;
  }
}

XBee::Checksum::Checksum(boost::asio::buffers_iterator<BufferSequence> begin, boost::asio::buffers_iterator<BufferSequence> end)
{
  calculate_checksum(begin, end);
}

XBee::Checksum::Checksum(const ConstBuffer &buffer)
{
  boost::asio::const_buffers_1 temp(buffer);
  calculate_checksum(buffers_begin(temp), buffers_end(temp));
}

template <typename BufferSequence>
void XBee::Checksum::calculate_checksum(boost::asio::buffers_iterator<BufferSequence> begin, boost::asio::buffers_iterator<BufferSequence> end)
{
  checksum_ = 0xFF;
  for(boost::asio::buffers_iterator<BufferSequence> it = begin; it != end; ++it) checksum_ -= *it;
}

template <> std::size_t serialize(const MutableBuffer &buffer, const XBee::SendHeader &header)
{
  OStream stream(buffer);
  stream.next(header.frame_id);
  if (header.destination_address.is64Bit()) {
    stream.next(uint8_t((header.destination_address.address >> 40) & 0xFF));
    stream.next(uint8_t((header.destination_address.address >> 32) & 0xFF));
    stream.next(uint8_t((header.destination_address.address >> 24) & 0xFF));
    stream.next(uint8_t((header.destination_address.address >> 16) & 0xFF));
    stream.next(uint8_t((header.destination_address.address >> 8) & 0xFF));
    stream.next(uint8_t(header.destination_address.address & 0xFF));
  } else {
    stream.next(uint8_t((header.destination_address.address >> 8) & 0xFF));
    stream.next(uint8_t(header.destination_address.address & 0xFF));
  }
  stream.next(header.options);
  return stream.getSize();
}

template <> std::size_t serializationLength(const XBee::SendHeader& header)
{
  return header.destination_address.is64Bit() ? 8 : 4;
}

template <> std::size_t serialize(const MutableBuffer &buffer, const XBee::ATCommand &command)
{
  OStream stream(buffer);
  stream.next(command.frame_id);
  stream.next(command.command[0]);
  stream.next(command.command[1]);
  std::copy(command.value.begin(), command.value.end(), stream.advance(command.value.size()));
  return stream.getSize();
}

template <> std::size_t serializationLength(const XBee::ATCommand &command)
{
  return 3 + command.value.size();
}

template <> std::size_t deserialize(const ConstBuffer& buffer, XBee::ReceiveHeader& header)
{
  IStream stream(buffer);
  stream.next(header.frame_id);
  if (header.source_address.is64Bit()) {
    uint8_t temp;
    header.source_address.address = 0;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 40;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 32;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 24;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 16;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 8;
    stream.next(temp); header.source_address.address |= uint64_t(temp);
  } else {
    uint8_t temp;
    header.source_address.address = 0;
    stream.next(temp); header.source_address.address |= uint64_t(temp) << 8;
    stream.next(temp); header.source_address.address |= uint64_t(temp);
  }
  stream.next(header.rssi);
  stream.next(header.options);
  return stream.getSize();
}

template <> std::size_t deserialize(const ConstBuffer& buffer, XBee::Status& status)
{
  IStream stream(buffer);
  stream.next(status.frame_id);
  stream.next(status.status);
  return stream.getSize();
}

template <> std::size_t deserialize(const ConstBuffer& buffer, XBee::ATResponse& response)
{
  IStream stream(buffer);
  stream.next(response.frame_id);
  stream.next(response.command[0]);
  stream.next(response.command[1]);
  stream.next(response.status);
  response.value.resize(stream.getLength());
  std::copy(stream.getData(), stream.getData() + stream.getLength(), response.value.begin());
  stream.advance(response.value.size());
  return stream.getSize();
}

} // namespace hector_serialization
