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
#include <thread>

#include <boost/asio/deadline_timer.hpp>

#include "fast_stream.h"

namespace legtool {

namespace {
const int64_t kBlockSize = (1 << 20);
const double kFlushTimeout_s = 1.0;

class ThreadWriter : boost::noncopyable {
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

  void Write(const std::string& data) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);
    service_.post(std::bind(&ThreadWriter::HandleWrite, this, data));
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

  void HandleWrite(const std::string& data) {
    BOOST_ASSERT(std::this_thread::get_id() == thread_.get_id());

    data_.push_back(data);

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

    size_t result =
        ::fwrite(data_.front().data(), data_.front().size(), 1, fd_);
    if (result == 0) {
      throw boost::system::system_error(
          boost::system::error_code(
              errno, boost::system::generic_category()));
    }

    child_offset_ += data_.front().size();
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
  std::list<std::string> data_;
  bool started_ = false;
  int64_t child_offset_ = 0;
  int64_t last_fadvise_ = 0;
  char buf_[65536] = {};

  // All threads.
  std::thread thread_;
};

}

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

  FastOStringStream ostr;
  TelemetryWriteStream<FastOStringStream> stream(ostr);
  stream.Write(static_cast<uint16_t>(TelemetryFormat::BlockType::kBlockSchema));
  stream.Write(static_cast<uint32_t>(ostr_schema.str().size()));
  stream.RawWrite(ostr_schema.str().data(), ostr_schema.str().size());

  impl_->writer_->Write(ostr.str());
}

void TelemetryLog::WriteData(uint32_t identifier,
                             uint32_t block_data_flags,
                             const std::string& serialized_data) {
  FastOStringStream ostr;
  TelemetryWriteStream<FastOStringStream> stream(ostr);
  stream.Write(static_cast<uint16_t>(TelemetryFormat::BlockType::kBlockData));
  stream.Write(static_cast<uint32_t>(serialized_data.size() + 4));
  stream.Write(block_data_flags);
  stream.RawWrite(serialized_data.data(), serialized_data.size());

  impl_->writer_->Write(ostr.str());
}

void TelemetryLog::WriteBlock(TelemetryFormat::BlockType block_type,
                              const std::string& data) {
  FastOStringStream ostr;
  TelemetryWriteStream<FastOStringStream> stream(ostr);
  stream.Write(static_cast<uint16_t>(block_type));
  stream.Write(static_cast<uint32_t>(data.size()));
  stream.RawWrite(data.data(), data.size());

  impl_->writer_->Write(ostr.str());
}

}
