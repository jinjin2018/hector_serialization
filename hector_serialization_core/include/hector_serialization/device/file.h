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

#ifndef HECTOR_SERIALIZATION_DEVICE_FILE_H
#define HECTOR_SERIALIZATION_DEVICE_FILE_H

#include <hector_serialization/device.h>
#include <boost/asio/io_service.hpp>
#include <boost/thread.hpp>

namespace hector_serialization {

class File
{
public:
  File(const std::string& filename = std::string());
  virtual ~File();

  virtual bool open() = 0;
  bool isOpen() const;
  void close();

  File& setFilename(const std::string& filename) { filename_ = filename; return *this; }
  const std::string& getFilename() const { return filename_; }

protected:
  class Worker;
  boost::shared_ptr<Worker> worker_;

  int fd_;
  std::string filename_;
};

class InputFile : public Device, virtual public File
{
public:
  InputFile(const std::string& filename = std::string(), std::size_t buffer_size = 8192);
  virtual ~InputFile();

  virtual bool open();
  using File::isOpen;
  using File::close;

  InputFile& setIgnoreEof(bool ignore_eof) { ignore_eof_ = ignore_eof; return *this; }

private:
  StreamBuf in_;
  boost::mutex read_mutex_;
  boost::condition read_condition_;

  void doRead();

  bool ignore_eof_;
};

class OutputFile : public Device, virtual public File
{
public:
  OutputFile(const std::string& filename = std::string(), std::size_t buffer_size = 8192);
  virtual ~OutputFile();

  virtual bool open();
  using File::isOpen;
  using File::close;

  using ChannelElement::write;
  virtual bool write(const BufferSequence &sequence);

private:
  StreamBuf out_;
  boost::mutex write_mutex_;
  boost::condition write_condition_;

  void doWrite();
};

} // namespace hector_serialization

#endif // HECTOR_SERIALIZATION_DEVICE_FILE_H
