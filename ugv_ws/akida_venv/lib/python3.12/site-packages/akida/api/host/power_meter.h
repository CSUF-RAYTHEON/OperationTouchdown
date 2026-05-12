#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <set>
#include <vector>

#include "host/circular_queue.h"
#include "host/soc_clock_mode.h"

namespace akida {

class PowerMeter {
 public:
  // the period at which power is updated by INA controller (140.8 ms)
  static constexpr int64_t ina_period_ms = 141;

  struct PowerEvent {
    explicit PowerEvent(int64_t t = 0, uint32_t v = 0, uint32_t i = 0)
        : ts(t), voltage(v), current(i) {}

    int64_t ts;        // Milliseconds
    uint32_t voltage;  // microVolt (10e-6)
    uint32_t current;  // milliAmpere (10e-3)
  };

  PowerMeter();
  // Log a new event
  void log_event(PowerEvent&& event);
  // Get all current events
  std::vector<PowerEvent> events();
  // Check if there is events
  bool has_events();
  // Get the current floor power in milliWatt
  float floor();
  // Reset the floor power at the specified time
  void reset(int64_t time);
  // Get PowerMeter instance
  static PowerMeter* get();

 private:
  CircularQueue<PowerEvent> events_;
  float floor_;
  int64_t next_update_;
  std::mutex events_mutex_;
};

using PowerMeterPtr = std::shared_ptr<PowerMeter>;

}  // namespace akida
