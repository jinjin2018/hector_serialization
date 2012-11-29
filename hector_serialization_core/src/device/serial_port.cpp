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
//==========================================

#include <hector_serialization/device/serial_port.h>

namespace hector_serialization {

  SerialPort::SerialPort(std::size_t buffer_size)
    : AsyncDevice<boost::asio::serial_port>(port_, buffer_size)
    , port_(io_service_)
  {
  }

  SerialPort::SerialPort(const std::string& device, std::size_t buffer_size)
    : AsyncDevice<boost::asio::serial_port>(port_, buffer_size)
    , port_(io_service_)
  {
    setDevice(device);
  }

  SerialPort::~SerialPort()
  {}

  bool SerialPort::open() {
    port_.open(device_);
    AsyncDevice<boost::asio::serial_port>::open();
    return isOpen();
  }

  SerialPort& SerialPort::setDevice(const std::string& device) {
    device_ = device;
    return *this;
  }
  const std::string& SerialPort::getDevice() {
    return device_;
  }

  SerialPort& SerialPort::setBaudrate(unsigned int rate) {
    port_.set_option(boost::asio::serial_port_base::baud_rate(rate));
    return *this;
  }
  unsigned int SerialPort::getBaudrate() {
    boost::asio::serial_port_base::baud_rate baud_rate;
    port_.get_option(baud_rate);
    return baud_rate.value();
  }

  SerialPort& SerialPort::setFlowControl(const flow_control_type& type) {
    port_.set_option(boost::asio::serial_port_base::flow_control(type));
    return *this;
  }
  SerialPort::flow_control_type SerialPort::getFlowControl() {
    boost::asio::serial_port_base::flow_control flow_control;
    port_.get_option(flow_control);
    return flow_control.value();
  }

  SerialPort& SerialPort::setParity(const parity_type& parity) {
    port_.set_option(boost::asio::serial_port_base::parity(parity));
    return *this;
  }
  SerialPort::parity_type SerialPort::getParity() {
    boost::asio::serial_port_base::parity parity;
    port_.get_option(parity);
    return parity.value();
  }

  SerialPort& SerialPort::setStopBits(const stop_bits_type& stop_bits) {
    port_.set_option(boost::asio::serial_port_base::stop_bits(stop_bits));
    return *this;
  }
  SerialPort::stop_bits_type SerialPort::getStopBits() {
    boost::asio::serial_port_base::stop_bits stop_bits;
    port_.get_option(stop_bits);
    return stop_bits.value();
  }

  SerialPort& SerialPort::setCharacterSize(unsigned int character_size) {
    port_.set_option(boost::asio::serial_port_base::character_size(character_size));
    return *this;
  }
  unsigned int SerialPort::getCharacterSize() {
    boost::asio::serial_port_base::character_size character_size;
    port_.get_option(character_size);
    return character_size.value();
  }

} // namespace hector_serialization
