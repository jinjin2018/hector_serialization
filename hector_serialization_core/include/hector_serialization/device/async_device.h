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

#ifndef HECTOR_SERIALIZATION_ASYNC_DEVICE_H
#define HECTOR_SERIALIZATION_ASYNC_DEVICE_H

#include <hector_serialization/device.h>
#include <hector_serialization/channel.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>

namespace hector_serialization {

template <typename StreamT>
class AsyncDevice : public Device
{
protected:
  boost::asio::io_service io_service_;

private:
  class Worker
  {
  public:
    Worker(AsyncDevice<StreamT> *owner, StreamT& stream);
    ~Worker();

    void write(const MutableBuffers1 &buffer);
    void cancel();

  private:
    void doRead();
    void readEnd(const boost::system::error_code&, std::size_t);
    void writeEnd(const boost::system::error_code&, std::size_t);
    void doClose();

    AsyncDevice<StreamT> *owner_;
    StreamT& stream_;

    boost::shared_ptr<boost::thread> background_thread_;
    std::vector<uint8_t> read_buffer_;
    bool stopping_;
  };
  friend class Worker;

  boost::shared_ptr<Worker> worker_;

public:
  AsyncDevice(StreamT& stream, std::size_t buffer_size = 8192);
  virtual ~AsyncDevice();

  void start();
  void stop();

  virtual bool open();
  virtual bool isOpen() const;
  virtual void close();
  virtual std::string getErrorMessage() const;

  using ChannelElement::write;
  bool write(const BufferSequence &sequence, const Context& context);
  using ChannelElement::wait;
  bool wait(const boost::posix_time::time_duration &timeout);

  StreamBuf *stream();

private:
  StreamT& stream_;
  StreamBuf in_;
  StreamBuf out_;

  boost::mutex write_mutex_;
  boost::condition write_condition_;
  boost::mutex read_mutex_;
  boost::condition read_condition_;

protected:
  boost::system::error_code error_code_;
};

template <typename StreamT>
AsyncDevice<StreamT>::AsyncDevice(StreamT& stream, std::size_t buffer_size)
  : stream_(stream)
  , in_(buffer_size)
  , out_(buffer_size)
{
}

template <typename StreamT>
AsyncDevice<StreamT>::~AsyncDevice()
{
}

template <typename StreamT>
void AsyncDevice<StreamT>::start()
{
  worker_.reset(new Worker(this, stream_));
}

template <typename StreamT>
void AsyncDevice<StreamT>::stop()
{
  // delete the worker (waits until all operations are finished)
  worker_.reset();
}

template <typename StreamT>
bool AsyncDevice<StreamT>::open()
{
//  stream_.open(error_code_);
//  if (error_code_) return false;
  start();
  return true;
}

template <typename StreamT>
bool AsyncDevice<StreamT>::isOpen() const
{
  return stream_.is_open();
}

template <typename StreamT>
void AsyncDevice<StreamT>::close()
{
  stop();
  stream_.close();
}

template <typename StreamT>
std::string AsyncDevice<StreamT>::getErrorMessage() const
{
  return error_code_ ? error_code_.message() : std::string();
}

template <typename StreamT>
AsyncDevice<StreamT>::Worker::Worker(AsyncDevice<StreamT> *owner, StreamT& stream)
  : owner_(owner)
  , stream_(stream)
  , read_buffer_(owner->in_.max_size())
  , stopping_(false)
{
  owner_->io_service_.post(boost::bind(&Worker::doRead, this));
  background_thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &owner_->io_service_)));
}

template <typename StreamT>
AsyncDevice<StreamT>::Worker::~Worker()
{
  cancel();
  background_thread_->join();
}

template <typename StreamT>
bool AsyncDevice<StreamT>::write(const BufferSequence &buffers, const Context& context) {
  if (!worker_) return false;

  boost::mutex::scoped_lock lock(write_mutex_);
  buffers_iterator<BufferSequence> begin = buffers_begin(buffers);
  buffers_iterator<BufferSequence> end   = buffers_end(buffers);
  std::size_t size = end - begin;

  try {
    MutableBuffers1 out_buffer = out_.prepare(size);
    std::copy(begin, end, buffers_begin(out_buffer));
    out_.commit(size);
    worker_->write(out_buffer);
  } catch (std::length_error& e) {
    return false;
  }

  return true;
}

template <typename StreamT>
bool AsyncDevice<StreamT>::wait(const boost::posix_time::time_duration &timeout)
{
  boost::mutex::scoped_lock lock(read_mutex_);
  return read_condition_.timed_wait(lock, timeout);
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::cancel() {
  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doClose, this));
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::doRead()
{
  try {
    stream_.async_read_some(MutableBuffers1(read_buffer_.data(), read_buffer_.size()), boost::bind(&Worker::readEnd, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    return;
  } catch(...) {
  }

  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doRead, this));
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::readEnd(const boost::system::error_code& error, std::size_t bytes_transfered)
{
  if (error) {
    // do something

  } else if (bytes_transfered > 0) {
//    owner_->read_mutex_.lock();
//    std::copy(read_buffer_.data(), read_buffer_.data() + bytes_transfered, buffers_begin(owner_->in_.prepare(bytes_transfered)));
//    // owner_->in_.commit(copy(boost::asio::const_buffers_1(read_buffer_.data(), bytes_transfered), owner_->in_.prepare(bytes_transfered)));
//    owner_->in_.commit(bytes_transfered);
//    owner_->read_mutex_.unlock();

//    owner_->trigger();
    owner_->handle(ConstBuffers1(read_buffer_.data(), bytes_transfered));
    owner_->read_condition_.notify_all();
  }

  // read_mutex_.unlock();
  if (!stopping_) owner_->io_service_.post(boost::bind(&Worker::doRead, this));
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::write(const MutableBuffers1 &buffer)
{
//  boost::mutex::scoped_lock lock(owner_->write_mutex_);
//  if (owner_->out_.size() == 0) return;

//  if (debug >= 2) {
//    std::cout << "sent " << out_.size() << " bytes:" << std::endl;
//    for(std::vector<unsigned char>::iterator it = out_.begin(); it != out_.end(); ++it) std::cout << std::hex << static_cast<unsigned int>(*it) << " ";
//    std::cout << std::dec << std::endl;
//  }

  boost::asio::async_write(stream_, buffer, boost::bind(&Worker::writeEnd, this, boost::asio::placeholders::error,
boost::asio::placeholders::bytes_transferred));
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::writeEnd(const boost::system::error_code& error, std::size_t bytes_transfered)
{
  if (error) {
    // do something
    std::cerr << "send: " << error.message() << std::endl;

  } else if (bytes_transfered > 0) {
    boost::mutex::scoped_lock lock(owner_->write_mutex_);
    owner_->out_.consume(bytes_transfered);
    if (owner_->out_.size() == 0)
      owner_->write_condition_.notify_all();
  }
}

template <typename StreamT>
void AsyncDevice<StreamT>::Worker::doClose()
{
  stopping_ = true;
  boost::system::error_code error;
  stream_.cancel(error);
}

template <typename StreamT>
StreamBuf *AsyncDevice<StreamT>::stream()
{
  return &in_;
}

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_ASYNC_DEVICE_H
