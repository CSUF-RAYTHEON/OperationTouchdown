/*******************************************************************************
 * Copyright 2019 Brainchip Holdings Ltd.
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

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "akida/hardware_component.h"
#include "akida/ip_version.h"
#include "akida/layer_params.h"
#include "akida/learning_params.h"
#include "akida/shape.h"
#include "akida/variables.h"
#include "infra/exports.h"

namespace akida {

class Layer;

/**
 * @brief A shared pointer to a Layer object
 */
using LayerPtr = std::shared_ptr<Layer>;

/**
 * @brief A shared pointer to a const Layer object
 */
using LayerConstPtr = std::shared_ptr<const Layer>;

/**
 * class Layer
 *
 * Public interface to an Akida Layer.
 *
 */

class AKIDASHAREDLIB_EXPORT Layer {
 public:
  using Shapes = std::vector<Shape>;

  /**
   * @brief Create a layer from a parameter structure and a name
   * @param params : structure to initialize the layer
   * @param name : name for the layer
   * @return a LayerPtr object
   */
  static LayerPtr create(const LayerParams* params, const std::string& name);

  virtual ~Layer() {}

  /**
   * @brief Returns the name of this layer
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief Set the name of this layer
   */
  virtual void set_name(const std::string& new_name) = 0;

  /**
   * @brief Get the default name of this layer
   */
  virtual std::string get_default_name() const = 0;

  /**
   * @brief Returns the input dimensions of this layer
   */
  virtual Shapes input_dimensions() const = 0;

  /**
   * @brief Returns the output dimensions of this layer
   */
  virtual Shape output_dimensions() const = 0;

  /**
   * @brief Returns the input bitwidth of this layer
   */
  virtual uint8_t input_bits() const = 0;

  /**
   * @brief Returns the output bitwidth of this layer
   */
  virtual uint8_t output_bits() const = 0;

  /**
   * @brief Returns true if output in this layer is signed
   */
  virtual bool output_signed() const = 0;

  /**
   * @brief Returns the IP version of the layers
   */
  virtual IpVersion ip_version() const = 0;

  /**
   * @brief Returns true if the layer can be mapped on the NP type
   */
  virtual bool is_mappable_on(hw::BasicType type) const = 0;

  /**
   * @brief Returns true if the layer is splittable on mapping
   */
  virtual bool is_splittable() const = 0;

  /**
   * @brief Returns true if the layer is a depthwise
   */
  virtual bool is_depthwise() const = 0;

  /**
   * @brief Returns true if this layer should be the first layer of a model
   */
  virtual bool is_input_layer() const = 0;

  /**
   * @brief Returns the parameters of this layer
   */
  virtual const LayerParams* params() const = 0;

  /**
   * @brief Returns the learning parameters of this layer
   */
  virtual const LearningParams* learning() const = 0;

  /**
   * @brief Returns true if this layer can learn
   */
  virtual bool can_learn() const = 0;

  /**
   * @brief Returns the Variables object attached to this layer
   */
  virtual Variables* variables() = 0;

  /**
   * @brief Returns the Variables object attached to this layer
   */
  virtual const Variables& variables() const = 0;

  /**
   * @brief Returns true if layer has an activation function
   */
  virtual bool has_activation() const = 0;

  /**
   * @brief Returns true if layer activation is equal to value
   * @param value : the activation to compare
   */
  virtual bool has_activation(ActivationType value) const = 0;

  /**
   * @brief Returns true if layer has a pooling operation
   */
  virtual bool has_pooling() const = 0;

  /**
   * @brief Returns true if layer pooling is equal to value
   * @param value : the pooling to compare
   */
  virtual bool has_pooling(PoolType value) const = 0;

  /**
   * @brief Returns the inbound layers of this layer
   */
  virtual std::vector<LayerConstPtr> inbound_layers() const = 0;
  virtual std::vector<LayerPtr> inbound_layers() = 0;

  /**
   * @brief Returns the outbound layers of this layer
   */
  virtual std::vector<LayerConstPtr> outbound_layers() const = 0;
  virtual std::vector<LayerPtr> outbound_layers() = 0;

  /**
   * The mapping of a Layer on one or more Neural Processors
   */
  struct Mapping {
    virtual ~Mapping() = default;
    std::shared_ptr<std::vector<hw::Component>> components;
  };

  using MappingPtr = std::shared_ptr<Mapping>;
  using MappingConstPtr = std::shared_ptr<const Mapping>;

  /**
   * Return the layer hardware mapping
   */
  virtual MappingConstPtr mapping() const = 0;
  virtual MappingPtr mapping() = 0;

  /**
   * @brief Returns the number of MACs for the layer.
   */
  virtual uint64_t get_macs() const = 0;
};

}  // namespace akida
