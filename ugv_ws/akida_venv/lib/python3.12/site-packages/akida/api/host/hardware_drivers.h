#pragma once

#include "infra/exports.h"
#include "infra/hardware_driver.h"

#include <memory>
#include <vector>

namespace akida {

/**
 * @brief Return a singleton vector containing all usable hardware drivers
 */
AKIDASHAREDLIB_EXPORT const std::vector<std::unique_ptr<HardwareDriver>>&
get_drivers();

}  // namespace akida
