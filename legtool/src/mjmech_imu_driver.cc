// Copyright 2014-2015 Josh Pieper, jjp@pobox.com.  All rights reserved.
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

#include "mjmech_imu_driver.h"

#include <linux/i2c-dev.h>

#include <thread>

#include "fail.h"

namespace legtool {

namespace {
int ErrWrap(int value) {
  if (value < 0) {
    throw SystemError(errno, boost::system::generic_category());
  }
  return value;
}
}

class MjmechImuDriver::Impl : boost::noncopyable {
 public:
  Impl(MjmechImuDriver* parent, boost::asio::io_service& service)
      : parent_(parent),
        service_(service),
        work_(service_),
        parent_id_(std::this_thread::get_id()) {}

  ~Impl() {
    done_ = true;
    if (child_.joinable()) { child_.join(); }
  }

  void AsyncStart(ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    // Capture our parent's parameters before starting our thread.
    parameters_ = parent_->parameters_;

    child_ = std::thread(std::bind(&Impl::Run, this, handler));
  }

 private:
  void Run(ErrorHandler handler) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    // TODO jpieper: Set the bit rate of the I2C device.

    try {
      // Open the I2C device and configure it.
      fd_ = ::open(parameters_.i2c_device.c_str(), O_RDWR);
      if (fd_ < 0) {
        throw SystemError(errno, boost::system::generic_category(),
                          "error opening '" + parameters_.i2c_device + "'");
      }

      uint8_t whoami[1] = {};

      ReadGyro(0x20, 1, whoami, sizeof(whoami));
      if (whoami[0] != 0xb1) {
        throw SystemError::einval(
            (boost::format("Incorrect whoami got 0x%02X expected 0xb1") %
             static_cast<int>(whoami[0])).str());
      }

      uint8_t ignored[1] = {};
      ReadAccel(0x20, 1, ignored, sizeof(ignored));

      // Configure the gyroscope.
      WriteGyro(0x00, {0xff}); // FS=250dps PWR=Normal XYZ=EN
      WriteGyro(0x01, {0x18}); // BW=22Hz
      WriteGyro(0x02, {99}); // ODR=100Hz

      // Configure the accelerometer.
      WriteAccel(0x20, {0b01010111}); // ODR=100Hz, low_power=off all enabled
      WriteAccel(0x23, {0b10101000}); // BDU=(block) BLE=off FS=11 (8g) HR=1
    } catch (SystemError& e) {
      e.error_code().Append("when initializing IMU");
      service_.post(std::bind(handler, e.error_code()));
      return;
    }

    service_.post(std::bind(handler, ErrorCode()));

    try {
      DataLoop();
    } catch (SystemError& e) {
      // TODO jpieper: We should somehow log this or do something
      // useful.  For now, just re-raise.
      throw;
    }
  }

  void DataLoop() {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    auto to_int16 = [](uint8_t msb, uint8_t lsb) {
      int32_t result = msb * 256 + lsb;
      if (result > 32767) { result = result - 65536; }
      return static_cast<int16_t>(result);
    };

    // Loop reading things and emitting data.
    while (!done_) {
      // Read gyro values until we get a new one.
      uint8_t data[10] = {};
      for (int i = 0; i < 15; i++) {
        ReadGyro(0x22, 1, data, sizeof(data));
        if ((data[0] & 0x01) != 0) { break; }
      }

      ImuData imu_data;
      imu_data.timestamp =
          boost::posix_time::microsec_clock::universal_time();

      uint8_t gdata[10] = {};
      ReadGyro(0xa2, 7, gdata, sizeof(gdata));

      uint8_t adata[10] = {};
      ReadAccel(0xa7, 7, adata, sizeof(adata));

      const double kAccelSensitivity = 0.004 / 64;
      const double kGravity = 9.80665;

      imu_data.accel_mps2.x =
          to_int16(adata[2], adata[1]) * kAccelSensitivity * kGravity;
      imu_data.accel_mps2.y =
          to_int16(adata[4], adata[3]) * kAccelSensitivity * kGravity;
      imu_data.accel_mps2.z =
          to_int16(adata[6], adata[5]) * kAccelSensitivity * kGravity;

      const double kGyroSensitivity = 0.00875; // For 250dps full scale range

      imu_data.body_rate_deg_s.x =
          to_int16(gdata[2], gdata[1]) * kGyroSensitivity;
      imu_data.body_rate_deg_s.y =
          to_int16(gdata[4], gdata[3]) * kGyroSensitivity;
      imu_data.body_rate_deg_s.z =
          to_int16(gdata[6], gdata[5]) * kGyroSensitivity;

      service_.post(std::bind(&Impl::SendData, this, imu_data));
    }
  }

  void SendData(const ImuData& imu_data) {
    BOOST_ASSERT(std::this_thread::get_id() == parent_id_);

    parent_->imu_data_signal_(&imu_data);
  }

  void WriteGyro(uint8_t reg, const std::vector<uint8_t>& data) {
    WriteData(parameters_.gyro_address, reg, data);
  }

  void WriteAccel(uint8_t reg, const std::vector<uint8_t>& data) {
    WriteData(parameters_.accel_address, reg, data);
  }

  void ReadGyro(uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_size) {
    ReadData(parameters_.gyro_address, reg, length,
             buffer, buffer_size);
  }

  void ReadAccel(uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_size) {
    ReadData(parameters_.accel_address, reg, length,
             buffer, buffer_size);
  }

  void WriteData(uint8_t address, uint8_t reg,
                 const std::vector<uint8_t>& data) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());

    try {
      ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)));

      union i2c_smbus_data i2c_data;
      BOOST_ASSERT(data.size() <= 32);
      i2c_data.block[0] = data.size();
      for (size_t i = 0; i < data.size(); ++i) {
        i2c_data.block[i + 1] = data[i];
      }
      ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_WRITE, reg,
                                 I2C_SMBUS_I2C_BLOCK_BROKEN, &i2c_data));
    } catch (SystemError& se) {
      se.error_code().Append(boost::format("WriteData(0x%02x, 0x%02x, ...)") %
                             static_cast<int>(address) %
                             static_cast<int>(reg));
      throw;
    }
  }

  void ReadData(uint8_t address, uint8_t reg, std::size_t length,
                uint8_t* buffer, std::size_t buffer_length) {
    BOOST_ASSERT(std::this_thread::get_id() == child_.get_id());
    BOOST_ASSERT(buffer_length >= length);

    try {
      try {
        ErrWrap(::ioctl(fd_, I2C_SLAVE, static_cast<int>(address)));
      } catch (SystemError& se) {
        se.error_code().Append("when setting address");
        throw;
      }

      union i2c_smbus_data data;
      BOOST_ASSERT(length <= 255);
      data.block[0] = length;
      try {
        ErrWrap(::i2c_smbus_access(fd_, I2C_SMBUS_READ, reg,
                                   I2C_SMBUS_I2C_BLOCK_DATA, &data));
      } catch (SystemError& se) {
        se.error_code().Append("during transfer");
        throw;
      }

      if (data.block[0] != length) {
        throw SystemError(
            EINVAL, boost::system::generic_category(),
            (boost::format("asked for length %d got %d") %
             length % static_cast<int>(data.block[0])).str());
      }
      std::memcpy(buffer, &data.block[1], length);
    } catch (SystemError& se) {
      se.error_code().Append(boost::format("ReadData(0x%02x, 0x%02x, %d)") %
                             static_cast<int>(address) %
                             static_cast<int>(reg) %
                             length);
      throw;
    }
  }


  // From both.
  MjmechImuDriver* const parent_;
  boost::asio::io_service& service_;
  boost::asio::io_service::work work_;
  Parameters parameters_;

  const std::thread::id parent_id_;
  std::thread child_;
  bool done_ = false;

  // From child only.
  int fd_ = -1;
};

MjmechImuDriver::MjmechImuDriver(boost::asio::io_service& service)
    : impl_(new Impl(this, service)) {}
MjmechImuDriver::~MjmechImuDriver() {}

void MjmechImuDriver::AsyncStart(ErrorHandler handler) {
  impl_->AsyncStart(handler);
}

}
