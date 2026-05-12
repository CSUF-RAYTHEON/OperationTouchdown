#pragma once

#include "infra/hardware_driver.h"

namespace akida {
namespace soc {

// These clock modes correspond to frequency values for the Akida IP in the SoC
enum class ClockMode {
  Performance = 300000000,
  Economy = 100000000,
  LowPower = 5000000,
};

// Get clock mode of SoC currently connected
ClockMode get_clock_mode(HardwareDriver* driver);
// Set clock mode of SoC currently connected
void set_clock_mode(HardwareDriver* driver, const ClockMode& clock_mode);

}  // namespace soc
}  // namespace akida
