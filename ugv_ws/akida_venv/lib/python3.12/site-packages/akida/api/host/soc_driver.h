#pragma once

#include "host/power_meter.h"
#include "host/soc_clock_mode.h"

namespace akida {

// Interface class to define an API specific to a SoC driver (as opposed to an
// FPGA one)
class SocDriver {
 public:
  virtual ~SocDriver() = default;
  // Get/Set clock mode
  virtual soc::ClockMode get_clock_mode() = 0;
  virtual void set_clock_mode(const soc::ClockMode& clock_mode) = 0;
  // Toggle power measurement on or off
  virtual void toggle_power_measurement(bool enable) = 0;
  // Get power meter
  virtual PowerMeterPtr power_meter() = 0;
};

}  // namespace akida
