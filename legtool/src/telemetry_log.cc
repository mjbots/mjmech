// Copyright 2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "telemetry_log.h"

#include <fcntl.h>

#include <cstdio>
#include <list>
#include <mutex>
#include <thread>

#include <boost/asio/deadline_timer.hpp>

#include "circular_buffer.h"
#include "move_wrapper.h"

namespace legtool {

namespace {
const int64_t kBlockSize = (1 << 20);
const double kFlushTimeout_s = 1.0;
const size_t kBufferStartPadding = 16;
}

class TelemetryLog::ThreadWriter : boost::noncopyable {
 public:
  ThreadWriter(const std::string& name, bool realtime)
      : ThreadWriter(OpenName(name), realtime) {}

  ThreadWriter(int fd, bool realtime)
      : ThreadWriter(OpenFd(fd), realtime) {}

  ThreadWriter(FILE* fd, bool realtime)
      : service_(),
        timer_(service_),
        parent_id_(std::this_thread::get_id()),
        realtime_(realtime),
        fd_(fd),
        thread_(std::bind(&ThreadWriter::Run, this)) {
    BOOST_ASSERT(fd >= 0);
  }

  ~ThreadWriter() {
    service_.post([this]() { this->service_.stop(); });
    thread_.join();
    WriteAll();
    ::fclose(fd_);
  }

  void Flush() {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    service_.post(std::bind(&ThreadWriter::HandleFlush, this));
  }

  class WriteHandler {
   public:
    WriteHandler(WriteHandler&& rhs)
      : parent_(rhs.parent_), ostr_(std::move(rhs.ostr_)) {}
    WriteHandler(ThreadWriter* parent,
                 std::unique_ptr<OStream> ostr)
        : parent_(parent), ostr_(std::move(ostr)) {}

    WriteHandler(const WriteHandler&) = delete;

    void operator()() { parent_->HandleWrite(std::move(ostr_)); }

   private:
    ThreadWriter* const parent_;
    std::unique_ptr<OStream> ostr_;
  };

  void Write(std::unique_ptr<OStream> buffer) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    service_.post(move_handler(WriteHandler(this, std::move(buffer))));
  }

  std::unique_ptr<OStream> GetBuffer() {
    std::lock_guard<std::mutex> guard(buffers_mutex_);

    std::unique_ptr<OStream> result;
    if (buffers_.empty()) {
      result = std::unique_ptr<OStream>(new OStream());
    } else {
      result = std::move(buffers_.back());
      buffers_.pop_back();
    }
    result->data()->resize(kBufferStartPadding);
    result->set_start(kBufferStartPadding);
    return result;
  }

 private:
  static FILE* OpenName(const std::string& name) {
    FILE* result = ::fopen(name.c_str(), "wb");
    if (result == nullptr) {
      throw boost::system::system_error(
          boost::system::error_code(
              errno, boost::system::generic_category()));
    }
    return result;
  }

  static FILE* OpenFd(int fd) {
    FILE* result = ::fdopen(fd, "wb");
    if (result == nullptr) {
      throw boost::system::system_error(
          boost::system::error_code(
              errno, boost::system::generic_category()));
    }
    return result;
  }

  void Run() {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());

    ::setvbuf(fd_, buf_, _IOFBF, sizeof(buf_));

    if (realtime_) {
      StartTimer();
    }

    size_t result = ::fwrite(TelemetryFormat::kHeader,
             ::strlen(TelemetryFormat::kHeader), 1, fd_);
    if (result == 0) {
      throw boost::system::system_error(
          boost::system::error_code(
              errno, boost::system::generic_category()));
    }

    service_.run();
  }

  void StartTimer() {
    timer_.expires_from_now(ConvertSecondsToDuration(kFlushTimeout_s));
    timer_.async_wait(std::bind(&ThreadWriter::HandleTimer, this,
                                std::placeholders::_1));
  }

  void HandleTimer(boost::system::error_code ec) {
    if (ec == boost::asio::error::operation_aborted) { return; }

    StartTimer();

    ::fflush(fd_);
  }

  void HandleWrite(std::unique_ptr<OStream> buffer) {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());

    data_.push_back(std::move(buffer));

    // If we're not already started, try to do so.
    if (!started_) { StartWrite(); }
  }

  void MaybeStart() {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());

    if (started_) { return; }
    started_ = true;
    service_.post(std::bind(&ThreadWriter::StartWrite, this));
  }

  void StartWrite() {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());

    started_ = false;
    if (data_.empty()) { return; }

    WriteFront();

    // At block boundaries, let the OS know that we don't plan on
    // using this data anytime soon so as to not fill up the page
    // cache.
    if (realtime_) {
      if ((child_offset_ - last_fadvise_) > kBlockSize) {
        ::fflush(fd_);
        int64_t next_fadvise = last_fadvise_;
        while ((next_fadvise + kBlockSize) < child_offset_) {
          next_fadvise += kBlockSize;
        }
        ::posix_fadvise(fileno(fd_),
                        last_fadvise_,
                        next_fadvise - last_fadvise_,
                        POSIX_FADV_DONTNEED);
        last_fadvise_ = next_fadvise;
      }
    }

    MaybeStart();
  }

  void WriteAll() {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    while (!data_.empty()) {
      WriteFront();
    }
  }

  void HandleFlush() {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());
    ::fflush(fd_);
  }

  void WriteFront() {
    // NOTE: This can only be called by the parent thread during the
    // final write-out, after the child thread has stopped.

    const OStream& stream = *data_.front();

    const char* ptr = &(*stream.data())[stream.start()];
    size_t size = stream.data()->size() - stream.start();
    size_t result = ::fwrite(ptr, size, 1, fd_);
    if (result == 0) {
      throw boost::system::system_error(
          boost::system::error_code(
              errno, boost::system::generic_category()));
    }

    child_offset_ += size;
    {
      std::lock_guard<std::mutex> guard(buffers_mutex_);
      buffers_.push_back(std::move(data_.front()));
    }
    data_.pop_front();
  }

  // Parent items.
  boost::asio::io_service service_;
  boost::asio::deadline_timer timer_;
  std::thread::id parent_id_;
  const bool realtime_;

  // Initialized from parent, then only accessed from child.
  FILE* fd_;

  // Only accessed from child thread.
  circular_buffer<std::unique_ptr<OStream> > data_;
  bool started_ = false;
  int64_t child_offset_ = 0;
  int64_t last_fadvise_ = 0;
  char buf_[65536] = {};

  // All threads.
  std::thread thread_;
  std::mutex buffers_mutex_;
  std::vector<std::unique_ptr<OStream> > buffers_;
};

class TelemetryLog::Impl {
 public:

  bool realtime_ = false;
  std::unique_ptr<ThreadWriter> writer_;
};

TelemetryLog::TelemetryLog() : impl_(new Impl()) {}
TelemetryLog::~TelemetryLog() {}

void TelemetryLog::SetRealtime(bool value) { impl_->realtime_ = value; }

void TelemetryLog::Open(const std::string& filename) {
  impl_->writer_.reset(new ThreadWriter(filename, impl_->realtime_));
}

void TelemetryLog::Open(int fd) {
  impl_->writer_.reset(new ThreadWriter(fd, impl_->realtime_));
}

void TelemetryLog::Close() {
  impl_->writer_.reset();
}

void TelemetryLog::Flush() {
  impl_->writer_->Flush();
}

void TelemetryLog::Split(const std::string& name) {
  throw std::runtime_error("not implemented");
}

void TelemetryLog::WriteSchema(uint32_t identifier,
                               uint32_t block_schema_flags,
                               const std::string& record_name,
                               const std::string& schema) {
  FastOStringStream ostr_schema;
  TelemetryWriteStream<FastOStringStream> stream_schema(ostr_schema);
  stream_schema.Write(block_schema_flags);
  stream_schema.Write(record_name);
  stream_schema.RawWrite(schema.data(), schema.size());

  auto buffer = GetBuffer();

  TelemetryWriteStream<OStream> stream(*buffer);
  stream.Write(static_cast<uint16_t>(TelemetryFormat::BlockType::kBlockSchema));
  stream.Write(static_cast<uint32_t>(ostr_schema.str().size()));
  stream.RawWrite(ostr_schema.str().data(), ostr_schema.str().size());

  impl_->writer_->Write(std::move(buffer));
}

void TelemetryLog::WriteData(uint32_t identifier,
                             uint32_t block_data_flags,
                             const std::string& serialized_data) {
  auto buffer = GetBuffer();

  TelemetryWriteStream<OStream> stream(*buffer);
  stream.Write(static_cast<uint16_t>(TelemetryFormat::BlockType::kBlockData));
  stream.Write(static_cast<uint32_t>(serialized_data.size() + 4));
  stream.Write(block_data_flags);
  stream.RawWrite(serialized_data.data(), serialized_data.size());

  impl_->writer_->Write(std::move(buffer));
}

void TelemetryLog::WriteBlock(TelemetryFormat::BlockType block_type,
                              const std::string& data) {
  auto buffer = GetBuffer();

  TelemetryWriteStream<OStream> stream(*buffer);
  stream.Write(static_cast<uint16_t>(block_type));
  stream.Write(static_cast<uint32_t>(data.size()));
  stream.RawWrite(data.data(), data.size());

  impl_->writer_->Write(std::move(buffer));
}

std::unique_ptr<TelemetryLog::OStream> TelemetryLog::GetBuffer() {
  return impl_->writer_->GetBuffer();
}

namespace {
class FakeStream : boost::noncopyable {
 public:
  FakeStream(char* data) : data_(data) {}

  void write(const char* data, size_t size) {
    std::memcpy(&data_[offset_], data, size);
    offset_ += size;
  }

 private:
  char* const data_;
  size_t offset_ = 0;
};
}

void TelemetryLog::WriteData(uint32_t identifier,
                             uint32_t block_data_flags,
                             std::unique_ptr<OStream> buffer) {
  const size_t kBlockHeaderSize = 2 + 4 + 4;
  BOOST_ASSERT(buffer->start() >= kBlockHeaderSize);

  size_t data_size = buffer->data()->size() - buffer->start();

  FakeStream stream(
      &(*buffer->data())[0] + buffer->start() - kBlockHeaderSize);
  TelemetryWriteStream<FakeStream> writer(stream);
  writer.Write(static_cast<uint16_t>(TelemetryFormat::BlockType::kBlockData));
  writer.Write(static_cast<uint32_t>(data_size + 4));
  writer.Write(block_data_flags);

  buffer->set_start(buffer->start() - kBlockHeaderSize);

  impl_->writer_->Write(std::move(buffer));
}

void TelemetryLog::WriteBlock(TelemetryFormat::BlockType block_type,
                              std::unique_ptr<OStream> buffer) {
  const size_t kBlockHeaderSize = 2 + 4;
  BOOST_ASSERT(buffer->start() >= kBlockHeaderSize);

  size_t data_size = buffer->data()->size() - buffer->start();

  FakeStream stream(
      &(*buffer->data())[0] + buffer->start() - kBlockHeaderSize);
  TelemetryWriteStream<FakeStream> writer(stream);
  writer.Write(static_cast<uint16_t>(block_type));
  writer.Write(static_cast<uint32_t>(data_size));
  buffer->set_start(buffer->start() - kBlockHeaderSize);

  impl_->writer_->Write(std::move(buffer));
}

}
