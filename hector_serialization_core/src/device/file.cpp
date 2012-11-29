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

#include <hector_serialization/device/file.h>
#include <boost/system/system_error.hpp>
#include <boost/system/error_code.hpp>

namespace hector_serialization {

class File::Worker : public boost::asio::io_service
{
public:
  Worker() : stopping_(false) {}
  ~Worker() { stop(); join(); }

  void start() { if (!thread_) thread_.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, this))); }
  void stop() { stopping_ = true; }
  bool isStopping() { return stopping_; }

  boost::shared_ptr<boost::thread> thread() { return thread_; }
  void join() { if (thread_) thread_->join(); }

private:
  boost::shared_ptr<boost::thread> thread_;
  bool stopping_;
};

File::File(const std::string &filename)
  : worker_(new Worker)
  , fd_(-1)
  , filename_(filename)
{
}

File::~File()
{}

bool File::isOpen() const
{
  return fd_ >= 0;
}

void File::close()
{
  if (fd_ >= 0) ::close(fd_);
  fd_ = -1;
}

InputFile::InputFile(const std::string &filename, std::size_t buffer_size)
  : File(filename)
  , in_(buffer_size)
{
  worker_->post(boost::bind(&InputFile::doRead, this));
  worker_->start();
}

InputFile::~InputFile()
{
  worker_.reset();
  close();
}

OutputFile::OutputFile(const std::string &filename, std::size_t buffer_size)
  : File(filename)
  , out_(buffer_size)
{
}

OutputFile::~OutputFile()
{
  worker_.reset();
  close();
}

bool InputFile::open()
{
  fd_ = ::open(filename_.c_str(), O_RDONLY, 0666);
  if (fd_ < 0) throw boost::system::system_error(errno, boost::system::get_system_category());
  return true;
}

bool OutputFile::open()
{
  fd_ = ::open(filename_.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd_ < 0) throw boost::system::system_error(errno, boost::system::get_system_category());
  return true;
}

bool OutputFile::write(const BufferSequence &sequence) {
  boost::mutex::scoped_lock lock(write_mutex_);

  std::size_t n = buffers_end(sequence) - buffers_begin(sequence);
  if (n == 0) return true;
  MutableBuffer buffer = out_.prepare(n);

  std::copy(buffers_begin(sequence), buffers_end(sequence), buffer_cast<char *>(buffer));
  out_.commit(n);

  worker_->post(boost::bind(&OutputFile::doWrite, this));
  worker_->start();

  return true;
}

void OutputFile::doWrite()
{
  boost::mutex::scoped_lock lock(write_mutex_);
  if (out_.size() == 0) return;

  if (::write(fd_, buffer_cast<const void *>(out_.data()), out_.size()) < 0) {
    throw boost::system::system_error(errno, boost::system::get_system_category());
  }
  out_.consume(out_.size());

  write_condition_.notify_all();
}

void InputFile::doRead()
{
  boost::mutex::scoped_lock lock(read_mutex_);

  MutableBuffer buffer = in_.prepare(in_.max_size() - in_.size());
  ssize_t n = ::read(fd_, buffer_cast<void*>(buffer), buffer_size(buffer));
  if (n < 0) {
    throw boost::system::system_error(errno, boost::system::get_system_category());
  }
  in_.commit(std::size_t(n));

  if (!worker_->isStopping()) worker_->post(boost::bind(&InputFile::doRead, this));
}

} // namespace hector_serialization
