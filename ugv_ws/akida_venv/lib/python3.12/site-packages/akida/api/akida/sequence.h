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

#include <memory>
#include <utility>

#include "akida/backend_type.h"
#include "akida/layer.h"
#include "akida/span.h"

namespace akida {
/**
 * class Sequence
 *
 * Represents a sequence of layers.
 * Sequences can be mapped in Software or on a Device.
 *
 */

class AKIDASHAREDLIB_EXPORT Sequence {
 public:
  virtual ~Sequence() {}

  /**
   * class Pass
   *
   * Represents a subset of the Sequence.
   * Hardware Sequences can typically be split into multiple passes on devices
   * that support hardware partial reconfiguration feature, reducing the
   * intervention of the software during inference.
   */
  struct Pass {
    virtual ~Pass() = default;
    std::vector<LayerPtr> layers;
  };

  using PassPtr = std::shared_ptr<Pass>;

  /**
   * @brief Returns a vector of pointers to passes in the Sequence
   * @return a vector containing pointers to Pass objects
   */
  virtual const std::vector<PassPtr>& passes() const = 0;

  /**
   * @brief Return the Sequence backend type
   *
   * @return : the Sequence BackendType
   */
  virtual BackendType backend() const = 0;

  /**
   * @brief Return the Sequence programming
   *
   * @return: an akida::span (pointer and size) referencing the serialized
   * program, or nullptr if the Sequence is not programmable
   */
  virtual akida::span<uint8_t> program() const = 0;

  struct ProgramParts {
    span<uint8_t> program_info;
    span<uint8_t> program_data;
  };

  /**
   * @brief Return the Sequence programming, split in 2 parts
   *
   * @return: a ProgramParts struct, representing parts of the program
   */
  virtual ProgramParts program_parts() const = 0;

  /**
   * @brief Return the Sequence name
   *
   * @return a string representing the sequence
   */
  virtual std::string name() const = 0;
};

using SequenceConstPtr = std::shared_ptr<const Sequence>;
using SequencePtr = std::shared_ptr<Sequence>;

}  // namespace akida
