/*******************************************************************************
 * Copyright 2021 Brainchip Holdings Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************
 */

#pragma once

#include <vector>

#include "akida/device.h"
#include "infra/exports.h"

namespace akida {

/**
 * @brief Return the full list of available hardware devices
 * @return vector of hardware devices found
 */
AKIDASHAREDLIB_EXPORT const std::vector<DevicePtr>& get_devices();

}  // namespace akida
