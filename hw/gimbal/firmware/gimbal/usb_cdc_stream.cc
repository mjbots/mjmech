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

#include "usb_cdc_stream.h"

#include <assert.h>

#include <cstring>

#include "usbd_cdc.h"

namespace {
UsbCdcStream* g_usb_cdc = nullptr;
bool g_init = false;

int8_t init_fs() {
  g_init = true;

  if (g_usb_cdc) {
    return g_usb_cdc->Init();
  }

  return 0;
}

int8_t deinit_fs() {
  g_init = false;

  if (g_usb_cdc) {
    return g_usb_cdc->Deinit();
  }

  return 0;
}

int8_t control_fs(uint8_t cmd, uint8_t* pbuf, uint16_t length) {
  if (!g_usb_cdc) { return 1; }

  return g_usb_cdc->HandleControl(cmd, pbuf, length);
}

int8_t receive_fs(uint8_t* pbuf, uint32_t* len) {
  if (!g_usb_cdc) { return 1; }

  return g_usb_cdc->HandleReceive(pbuf, len);
}

int8_t tx_complete_fs() {
  if (!g_usb_cdc) { return 1; }

  return g_usb_cdc->HandleTxComplete();
}
}

extern "C" {
extern USBD_HandleTypeDef hUsbDeviceFS;

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  init_fs,
  deinit_fs,
  control_fs,
  receive_fs,
  tx_complete_fs,
};

}

UsbCdcStream::UsbCdcStream() {
  // There should only be one of us at a time.
  assert(!g_usb_cdc);
  g_usb_cdc = this;
}

UsbCdcStream::~UsbCdcStream() {
  assert(g_usb_cdc);
  g_usb_cdc = nullptr;
}

int8_t UsbCdcStream::Init() {
  if (rx_callback_.valid()) {
    // We had a request before we were valid.  Kick it off now.
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_buffer_);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
  return 0;
}

int8_t UsbCdcStream::Deinit() {
  rx_position_ = 0;
  rx_size_ = 0;
  rx_queue_ = gsl::string_span();

  // If we had an RX callback, invoke it now with an error.
  if (rx_callback_.valid()) {
    auto callback = rx_callback_;
    rx_callback_ = SizeCallback();
    callback(1, 0);
  }

  tx_queue_ = gsl::cstring_span();
  tx_callback_size_ = 0;
  write_outstanding_ = false;

  // If we had a TX callback, invoke it now with an error.
  if (tx_callback_.valid()) {
    auto callback = tx_callback_;
    tx_callback_ = SizeCallback();
    callback(1, 0);
  }

  return 0;
}

void UsbCdcStream::AsyncReadSome(const gsl::string_span& buffer,
                           SizeCallback callback) {
  // Do we have any data in our buffer now?  If so, give what we can
  // and call it good.
  if (buffer.size() == 0) {
    callback(0, 0);
    return;
  }

  if (!g_init) {
    // We aren't active now.  Just queue the callback and don't start
    // anything.
    rx_callback_ = callback;
    rx_queue_ = buffer;
    return;
  }

  auto available = rx_size_ - rx_position_;
  if (available > 0) {
    auto to_read = std::min(available, buffer.size());
    std::memcpy(buffer.data(), &rx_buffer_[rx_position_], to_read);
    rx_position_ += to_read;
    callback(0, to_read);
  } else {
    // Nothing is available.  Mark our buffer as empty and request
    // another packet.
    rx_position_ = 0;
    rx_size_ = 0;
    rx_callback_ = callback;
    rx_queue_ = buffer;

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_buffer_);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
}

void UsbCdcStream::AsyncWriteSome(const gsl::cstring_span& buffer,
                            SizeCallback callback) {
  if (!g_init) {
    // Mark it all as read immediately.
    callback(0, buffer.size());
    return;
  }
  if (write_outstanding_) {
    // This should only happen as a result of flushing the buffer, as
    // it is ill-formed to call AsyncWriteSome while another request
    // is outstanding.
    assert(!tx_callback_.valid());

    tx_queue_ = buffer;
    tx_callback_ = callback;
    return;
  }

  // See how much we can fit in.
  auto available = sizeof(tx_buffer_) - tx_position_;
  auto to_read = std::min(available, buffer.size());
  std::memcpy(&tx_buffer_[tx_position_], buffer.data(), to_read);
  tx_position_ += to_read;

  if (tx_position_ >= sizeof(tx_buffer_)) {
    // We've filled up our buffer, kick off a transmit and make the
    // caller wait.
    write_outstanding_ = true;

    tx_callback_ = callback;
    tx_callback_size_ = to_read;

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buffer_, tx_position_);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  } else {
    // We'll count writing to the buffer the same as completing a
    // write, so just invoke the callback right now.
    //
    // TODO jpieper: This should probably be routed through an event
    // loop of some sort.
    callback(0, to_read);
  }
}

void UsbCdcStream::Poll() {
  if (tx_complete_) {
    tx_complete_ = false;

    write_outstanding_ = false;
    tx_position_ = 0;
    if (tx_queue_.size() != 0) {
      // We had a transmit request while we were flushing the buffer.
      // Send it now.
      gsl::cstring_span queue = tx_queue_;
      tx_queue_ = gsl::cstring_span();
      auto callback = tx_callback_;
      tx_callback_ = SizeCallback();
      AsyncWriteSome(queue, callback);
    } else if (tx_callback_.valid()) {
      auto callback = tx_callback_;
      tx_callback_ = SizeCallback();
      auto callback_size = tx_callback_size_;
      tx_callback_size_ = 0;
      callback(0, callback_size);
    }
  }
  if (rx_complete_) {
    rx_complete_ = false;

    // We better have a callback outstanding.
    if (rx_callback_.valid()) {
      assert(rx_queue_.size() != 0);

      auto to_read = std::min(static_cast<std::size_t>(rx_size_),
                              rx_queue_.size());
      std::memcpy(rx_queue_.data(), rx_buffer_, to_read);
      rx_queue_ = gsl::string_span();
      rx_position_ = to_read;
      auto callback = rx_callback_;
      rx_callback_ = SizeCallback();
      callback(0, to_read);
    }
  }
}

void UsbCdcStream::PollMillisecond() {
  // Flush our output buffer on millisecond boundaries.
  if (!write_outstanding_ && tx_position_ != 0) {
    write_outstanding_ = true;

    USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buffer_, tx_position_);
    USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  }
}

int8_t UsbCdcStream::HandleControl(uint8_t, uint8_t*, uint16_t) {
  return 0;
}

int8_t UsbCdcStream::HandleReceive(uint8_t* buf, uint32_t* len) {
  rx_size_ = *len;
  rx_complete_ = true;

  return 0;
}

int8_t UsbCdcStream::HandleTxComplete() {
  tx_complete_ = true;

  return 0;
}
