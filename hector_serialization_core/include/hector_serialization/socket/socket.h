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

#ifndef HECTOR_SERIALIZATION_SOCKET_ASYNC_SOCKET_H
#define HECTOR_SERIALIZATION_SOCKET_ASYNC_SOCKET_H

#include <hector_serialization/channel.h>
#include <hector_serialization/socket/context.h>

#include <boost/asio.hpp>
#include <boost/asio/basic_socket.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace hector_serialization {
namespace socket {

template <typename SocketT>
class DatagramSocket : public ChannelElement
{
public:
  typedef SocketT Socket;
  typedef typename SocketT::endpoint_type Endpoint;
  typedef typename SocketT::protocol_type Protocol;
  typedef boost::asio::ip::address Address;
  typedef EndpointContextElement<Endpoint> ContextElement;

private:
  class Worker
  {
  public:
    Worker(DatagramSocket<SocketT> *owner, SocketT& socket, std::size_t buffer_size);
    ~Worker();

    void send_to(const BufferSequence &sequence, Endpoint const &endpoint);
    void send(const BufferSequence &sequence);
    void cancel();

  private:
    void doRead();
    void readEnd(const boost::system::error_code&, std::size_t);
    void writeEnd(const boost::system::error_code&, std::size_t);
    void doClose();

    DatagramSocket<SocketT> *owner_;
    SocketT& socket_;

    boost::shared_ptr<boost::thread> background_thread_;
    std::vector<uint8_t> read_buffer_;
    bool stopping_;
  };
  friend class Worker;

  boost::shared_ptr<Worker> worker_;

public:
  DatagramSocket(SocketT& socket, std::size_t buffer_size = 8192);
  virtual ~DatagramSocket();

  void start();
  void stop();

  /* connect/bind socket */
  virtual bool open(const Protocol& protocol);
  virtual bool connect(const Endpoint& endpoint);
  virtual void disconnect();
  virtual bool bind(const Endpoint& endpoint);
  virtual bool isOpen() const;
  virtual void close();
  virtual std::string getErrorMessage() const;

  using ChannelElement::write;
  bool write(const BufferSequence &sequence, const Context& context);
  using ChannelElement::wait;
  bool wait(const boost::posix_time::time_duration &timeout);

//  StreamBuf *stream();

private:
  SocketT& socket_;
//  StreamBuf in_;
//  StreamBuf out_;
  std::size_t buffer_size_;

  boost::recursive_mutex write_mutex_;
  boost::condition write_condition_;
  boost::recursive_mutex read_mutex_;
  boost::condition read_condition_;

protected:
  boost::asio::io_service io_service_;
  ContextElement remote_context_;
  boost::system::error_code error_code_;
};

template <typename SocketT>
DatagramSocket<SocketT>::DatagramSocket(SocketT& socket, std::size_t buffer_size)
  : socket_(socket)
//  , in_(buffer_size)
//  , out_(buffer_size)
  , buffer_size_(buffer_size)
{
}

template <typename SocketT>
DatagramSocket<SocketT>::~DatagramSocket()
{
}

template <typename SocketT>
void DatagramSocket<SocketT>::start()
{
  worker_.reset(new Worker(this, socket_, buffer_size_));
}

template <typename SocketT>
void DatagramSocket<SocketT>::stop()
{
  // delete the worker (waits until all operations are finished)
  worker_.reset();
}

template <typename SocketT>
bool DatagramSocket<SocketT>::open(const Protocol& protocol = boost::asio::ip::udp::v4())
{
  socket_.open(protocol, error_code_);
  if (error_code_) return false;
  start();
  return true;
}

template <typename SocketT>
bool DatagramSocket<SocketT>::connect(const Endpoint& endpoint)
{
  socket_.connect(endpoint, error_code_);
  return !error_code_;
}

template <typename SocketT>
void DatagramSocket<SocketT>::disconnect()
{
}

template <typename SocketT>
bool DatagramSocket<SocketT>::bind(const Endpoint& endpoint)
{
  socket_.bind(endpoint, error_code_);
  return !error_code_;
}

template <typename SocketT>
bool DatagramSocket<SocketT>::isOpen() const
{
  return socket_.is_open();
}

template <typename SocketT>
void DatagramSocket<SocketT>::close()
{
  stop();
  socket_.close();
}

template <typename SocketT>
std::string DatagramSocket<SocketT>::getErrorMessage() const
{
  return error_code_ ? error_code_.message() : std::string();
}

template <typename SocketT>
DatagramSocket<SocketT>::Worker::Worker(DatagramSocket<SocketT> *owner, SocketT& socket, std::size_t buffer_size)
  : owner_(owner)
  , socket_(socket)
  , read_buffer_(buffer_size)
  , stopping_(false)
{
  owner_->io_service_.post(boost::bind(&Worker::doRead, this));
  background_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &owner_->io_service_)));
}

template <typename SocketT>
DatagramSocket<SocketT>::Worker::~Worker()
{
  cancel();
  background_thread_->join();
}

template <typename SocketT>
bool DatagramSocket<SocketT>::write(const BufferSequence &buffers, const Context& context) {
  if (!worker_) return false;

  ContextElement const *element = context.get<ContextElement>().get();
//  worker_->send(buffers, element ? &element->value : 0);

//  boost::mutex::scoped_lock lock(write_mutex_);
//  buffers_iterator<BufferSequence> begin = buffers_begin(buffers);
//  buffers_iterator<BufferSequence> end   = buffers_end(buffers);
//  std::size_t size = end - begin;

//  try {
//    MutableBuffer out_buffer = out_.prepare(size);
//    std::copy(begin, end, buffers_begin(out_buffer));
//    out_.commit(size);
//    if (element)
//      worker_->send_to(out_buffer, element->value);
//    else
//      worker_->send(out_buffer);
//  } catch (std::length_error& e) {
//    return false;
//  }

  try {
    if (element)
      worker_->send_to(buffers, element->endpoint());
    else
      worker_->send(buffers);
  } catch (...) {
    return false;
  }

  return true;
}

template <typename SocketT>
bool DatagramSocket<SocketT>::wait(const boost::posix_time::time_duration &timeout)
{
  boost::recursive_mutex::scoped_lock lock(read_mutex_);
  return read_condition_.timed_wait(lock, timeout);
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::cancel() {
  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doClose, this));
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::doRead()
{
  owner_->read_mutex_.lock();

  try {
    socket_.async_receive_from(MutableBuffers1(read_buffer_.data(), read_buffer_.size()), owner_->remote_context_.endpoint(), boost::bind(&Worker::readEnd, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    return;
  } catch(std::runtime_error& e) {
#ifndef NDEBUG
    std::cerr << "doRead(): " << e.what() << std::endl;
#endif
  }

  owner_->read_mutex_.unlock();
  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doRead, this));
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::readEnd(const boost::system::error_code& error, std::size_t bytes_transfered)
{
  if (error) {
#ifndef NDEBUG
    std::cerr << "readEnd(): " << error.message() << std::endl;
#endif
    // do something

  } else if (bytes_transfered > 0) {
//    owner_->read_mutex_.lock();
//    std::copy(read_buffer_.data(), read_buffer_.data() + bytes_transfered, buffers_begin(owner_->in_.prepare(bytes_transfered)));
//    // owner_->in_.commit(copy(boost::asio::const_buffers_1(read_buffer_.data(), bytes_transfered), owner_->in_.prepare(bytes_transfered)));
//    owner_->in_.commit(bytes_transfered);
//    owner_->read_mutex_.unlock();

    // owner_->trigger(Context(owner_->remote_context_));
    owner_->handle(ConstBuffers1(read_buffer_.data(), bytes_transfered), Context(owner_->remote_context_));
    owner_->read_condition_.notify_all();
  }

  owner_->read_mutex_.unlock();
  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doRead, this));
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::send_to(const BufferSequence &sequence, Endpoint const &endpoint)
{
  owner_->write_mutex_.lock();

//  boost::mutex::scoped_lock lock(owner_->write_mutex_);
//  if (owner_->out_.size() == 0) return;

//  if (debug >= 2) {
//    std::cout << "sent " << out_.size() << " bytes:" << std::endl;
//    for(std::vector<unsigned char>::iterator it = out_.begin(); it != out_.end(); ++it) std::cout << std::hex << static_cast<unsigned int>(*it) << " ";
//    std::cout << std::dec << std::endl;
//  }

//  socket_.async_send(owner_->out_.data(), boost::bind(&Worker::writeEnd, this, boost::asio::placeholders::error,
//                                                                  boost::asio::placeholders::bytes_transferred));
  socket_.async_send_to(sequence, endpoint, boost::bind(&Worker::writeEnd, this, boost::asio::placeholders::error,
                                                       boost::asio::placeholders::bytes_transferred));
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::send(const BufferSequence &sequence)
{
  owner_->write_mutex_.lock();
  socket_.async_send(sequence, boost::bind(&Worker::writeEnd, this, boost::asio::placeholders::error,
                                                       boost::asio::placeholders::bytes_transferred));
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::writeEnd(const boost::system::error_code& error, std::size_t bytes_transfered)
{
  if (error) {
    // do something
    std::cerr << "send: " << error.message() << std::endl;

  } else if (bytes_transfered > 0) {
//    boost::mutex::scoped_lock lock(owner_->write_mutex_);
//    owner_->out_.consume(bytes_transfered);
//    if (owner_->out_.size() == 0)
    owner_->write_condition_.notify_all();
  }
  owner_->write_mutex_.unlock();
}

template <typename SocketT>
void DatagramSocket<SocketT>::Worker::doClose()
{
  stopping_ = true;
  boost::system::error_code error;
  socket_.cancel(error);
}

} // namespace socket
} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_SOCKET_ASYNC_SOCKET_H
