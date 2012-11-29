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

#ifndef HECTOR_SERIALIZATION_SERIAL_PORT_H
#define HECTOR_SERIALIZATION_SERIAL_PORT_H

#include <hector_serialization/device/async_device.h>
#include <boost/asio/serial_port.hpp>

namespace hector_serialization {

class SerialPort : public AsyncDevice<boost::asio::serial_port>
{
public:
  SerialPort(std::size_t buffer_size = 8192);
  SerialPort(const std::string& device, std::size_t buffer_size = 8192);
  virtual ~SerialPort();

  bool open();

  SerialPort& setDevice(const std::string& device);
  const std::string& getDevice();

  SerialPort& setBaudrate(unsigned int rate);
  unsigned int getBaudrate();

  typedef boost::asio::serial_port_base::flow_control::type flow_control_type;
  SerialPort& setFlowControl(const flow_control_type& type);
  flow_control_type getFlowControl();

  typedef boost::asio::serial_port_base::parity::type parity_type;
  SerialPort& setParity(const parity_type& parity);
  parity_type getParity();

  typedef boost::asio::serial_port_base::stop_bits::type stop_bits_type;
  SerialPort& setStopBits(const stop_bits_type& stop_bits);
  stop_bits_type getStopBits();

  SerialPort& setCharacterSize(unsigned int character_size);
  unsigned int getCharacterSize();

  boost::asio::serial_port::native_type native() { return port_.native(); }

private:
  boost::asio::serial_port port_;
  std::string device_;
};

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_SERIAL_PORT_H
