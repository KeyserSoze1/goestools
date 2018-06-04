#include "limesdr_source.h"

#include <pthread.h>

#include <cassert>
#include <cstring>
#include <iostream>

std::unique_ptr<LimeSDR> LimeSDR::open(uint32_t index) {
  int n = 0;
  lms_info_str_t list[8]; //should be large enough to hold all detected devices
  if ((n = LMS_GetDeviceList(list)) < 0 || index >= n) {
      std::cerr
      << "Unable to open LimeSDR device: index invalid"
      << std::endl;
    exit(1);
  }

  lms_device_t* device = NULL;
  if (LMS_Open(&device, list[index], NULL)) {
    std::cerr
      << "Unable to open LimeSDR device"
      << std::endl;
  }
  return std::make_unique<LimeSDR>(device);
}

LimeSDR::LimeSDR(lms_device_t* dev) : dev_(dev) {
  LMS_Init(dev_);
  LMS_EnableChannel(dev_, LMS_CH_RX, 0, true);
}

LimeSDR::~LimeSDR() {
  if (dev_ != nullptr) {
    LMS_Close(dev_);
  }
}

void LimeSDR::setFrequency(uint32_t freq) {
  assert(dev_ != nullptr);
  auto rv = LMS_SetLOFrequency(dev_, LMS_CH_RX, 0, freq);
  assert(rv >= 0);
}

void LimeSDR::setSampleRate(uint32_t rate) {
  assert(dev_ != nullptr);
  auto rv = LMS_SetSampleRate(dev_, rate, 0);
  assert(rv >= 0);
}

uint32_t LimeSDR::getSampleRate() const {
  float_type host, rf;
  LMS_GetSampleRate(dev_, LMS_CH_RX, 0, &host, &rf);
  return host;
}

void LimeSDR::setGain(int gain) {
  assert(dev_ != nullptr);
  auto rv = LMS_SetGaindB(dev_, LMS_CH_RX, 0, gain);
  assert(rv >= 0);
}

void LimeSDR::start(const std::shared_ptr<Queue<Samples> >& queue) {
  assert(dev_ != nullptr);
  queue_ = queue;
  
  rx_stream_.channel = 0; //channel number
  rx_stream_.fifoSize = 1024 * 1024; //fifo size in samples
  rx_stream_.throughputVsLatency = 0.5; //some middle ground
  rx_stream_.isTx = false; //RX channel
  rx_stream_.dataFmt = lms_stream_t::LMS_FMT_F32; //32 bit floats
  auto rv = LMS_SetupStream(dev_, &rx_stream_);
  assert(rv >= 0);
  streaming_ = true;
  thread_ = std::thread([&] {
      const int buffersize = 1024 * 8; //complex samples per buffer
      float buffer[buffersize * 2]; //buffer to hold complex values (2*samples))
      lms_stream_meta_t rx_metadata;
      rx_metadata.flushPartialPacket = false; //currently has no effect in RX
      rx_metadata.waitForTimestamp = false; //currently has no effect in RX
      int samples_read;
      LMS_StartStream(&rx_stream_);
      while (streaming_) {
        samples_read = LMS_RecvStream(&rx_stream_, buffer, buffersize, &rx_metadata, 500);
        auto out = queue_->popForWrite();
        out->resize(samples_read);
        memcpy(out->data(), buffer, samples_read * sizeof(std::complex<float>));
        // Publish output if applicable
        if (samplePublisher_) {
          samplePublisher_->publish(*out);
        }

        queue_->pushWrite(std::move(out));
      }
    });
  
  pthread_setname_np(thread_.native_handle(), "limesdr");
}

void LimeSDR::stop() {
  streaming_ = false;
  assert(dev_ != nullptr);
  auto rv = LMS_StopStream(&rx_stream_);
  assert(rv >= 0);
  LMS_DestroyStream(dev_, &rx_stream_);

  // Wait for thread to terminate
  thread_.join();

  // Close queue to signal downstream
  queue_->close();

  // Clear reference to queue
  queue_.reset();
}
