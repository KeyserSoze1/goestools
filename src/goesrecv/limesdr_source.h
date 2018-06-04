#pragma once

#include <stdint.h>

#include <memory>
#include <thread>
#include <vector>

#include "lime/LimeSuite.h"

#include "source.h"

class LimeSDR : public Source {
public:
  static std::unique_ptr<LimeSDR> open(uint32_t index = 0);

  explicit LimeSDR(lms_device_t* dev);
  ~LimeSDR();

  void setFrequency(uint32_t freq);

  void setSampleRate(uint32_t rate);

  virtual uint32_t getSampleRate() const override;

  void setGain(int gain);

  void setSamplePublisher(std::unique_ptr<SamplePublisher> samplePublisher) {
    samplePublisher_ = std::move(samplePublisher);
  }

  virtual void start(const std::shared_ptr<Queue<Samples> >& queue) override;

  virtual void stop() override;

protected:
  lms_device_t* dev_;
  lms_stream_t rx_stream_;
  bool streaming_;

  // Background RX thread
  std::thread thread_;

  // Set on start; cleared on stop
  std::shared_ptr<Queue<Samples> > queue_;

  // Optional publisher for samples
  std::unique_ptr<SamplePublisher> samplePublisher_;
};
