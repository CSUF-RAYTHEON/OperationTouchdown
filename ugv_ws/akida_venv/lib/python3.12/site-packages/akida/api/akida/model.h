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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "akida/dense.h"
#include "akida/device.h"
#include "akida/layer.h"
#include "akida/map_constraints.h"
#include "akida/sequence.h"
#include "akida/shape.h"
#include "infra/exports.h"

namespace akida {

class Model;

/**
 * @brief A unique pointer to a Model object
 */
using ModelPtr = std::unique_ptr<Model>;

/**
 * class Model
 *
 * Public interface to an Akida Model. Users can create or load
 * models, save models, and propagate spike events through the full model.
 *
 */

class AKIDASHAREDLIB_EXPORT Model {
 public:
  virtual ~Model() = default;
  /**
   * @brief Create a model object
   */
  static ModelPtr create();

  /**
   * @brief Create a model from a serialized model
   * @param buffer : char buffer containing the model (a flatbuffer)
   * @param size : size of the buffer
   */
  static ModelPtr from_buffer(const char* buffer, size_t size);

  /**
   * @brief Add a layer to the current model
   *
   * A list of inbound layers can optionally be specified.
   *
   * These layers must already be included in the model.
   *
   * If no inbound layer is specified, and the layer is not the first layer in
   * the model, the last included layer will be used as inbound layer.
   *
   * @param layer : layer instance to be added to the model
   * @param inbound_layers : the inbound layers for this layer
   */
  virtual void add(LayerPtr layer,
                   const std::vector<LayerPtr> inbound_layers = {}) = 0;

  /**
   * @brief Remove the last layer of the current model
   */
  virtual void pop_layer() = 0;

  /**
   * @brief Return the serialized model configuration (all layers and weights)
   * as a vector of char (bytes).
   */
  virtual std::vector<char> to_buffer() = 0;

  /**
   * @brief Prepare the internal parameters of the last layer of the model for
   * training.
   * @param params : the LearningParams
   */
  virtual void compile(const LearningParams& params) = 0;

  /**
   * @brief Propagates inputs to train the model.
   * @param inputs       : pointer to Dense inputs
   * @param input_labels : integer value labels of the input classes,
   * for supervised learning
   * @param batch_size   : maximum number of inputs that should be processed at
   * a time. If 0, the whole input size will be taken.
   * @return Dense outputs from the model last layer
   */
  virtual DensePtr fit(DenseConstPtr inputs,
                       const std::vector<int32_t>& input_labels = {},
                       uint32_t batch_size = 0) = 0;

  /**
   * @brief Propagates events through the model
   * @param inputs       : pointer to Dense inputs
   * @param batch_size   : maximum number of inputs that should be processed at
   * a time. If 0, the whole input size will be taken.
   * @return Dense outputs from the model last layer
   */
  virtual DensePtr forward(DenseConstPtr inputs, uint32_t batch_size = 0) = 0;

  /**
   * @brief Evaluates the results of events propagation through the model
   *
   * This method propagates a set of inputs through the model and returns the
   * results in the form of a Tensor of float values.
   * It applies ONLY on models whithout an activation on the last layer.
   * The output values are obtained from the model discrete potentials by
   * applying a shift and a scale.
   *
   * @param inputs : Dense inputs to be processed by model
   * @param batch_size   : maximum number of inputs that should be processed at
   * a time. If 0, the whole input size will be taken.
   * @return rescaled output potentials from the model last layer
   */
  virtual DensePtr predict(DenseConstPtr inputs, uint32_t batch_size = 0) = 0;

  /**
   * @brief Maps the model to a Device
   *
   * This method tries to map a Model to the specified Device, with specified
   * constraints, implicitly identifying one or more layer sequences that are
   * mapped individually on the Device Mesh.
   *
   * An optional hw_only parameter can be specified to force the mapping
   * strategy to use only one hardware sequence, thus reducing software
   * intervention on the inference.
   *
   * An optional map constraints parameter can be specified to force the mapping
   * strategy.
   *
   * @param device: the target Device or nullptr
   * @param hw_only: when true, the model should be mapped in one sequence
   * @param constraints: the map constraints
   */
  virtual void map(DevicePtr device, bool hw_only = false,
                   MapConstraintsPtr constraints = nullptr) = 0;

  /**
   * @brief Returns a pointer to a layer from its index
   * @param index : index to the layer to retrieve
   * @return a pointer to a Layer object
   */
  virtual LayerPtr get_layer(size_t index) = 0;

  /**
   * @brief Returns a pointer to a layer from its name
   * @param name : name of the layer to retrieve
   * @return a pointer to a Layer object
   */
  virtual LayerPtr get_layer(const std::string& name) = 0;

  /**
   * @brief Returns a pointer to a layer from its index
   * @param index : index to the layer to retrieve
   * @return a const pointer to a Layer object
   */
  virtual LayerConstPtr get_layer(size_t index) const = 0;

  /**
   * @brief Returns a pointer to a layer from its name
   * @param name : name of the layer to retrieve
   * @return a const pointer to a Layer object
   */
  virtual LayerConstPtr get_layer(const std::string& name) const = 0;

  /**
   * @brief Returns a vector of pointers to layers in the model
   * @return a vector containing pointers to Layer objects
   */
  virtual const std::vector<LayerPtr>& get_layers() const = 0;

  /**
   * @brief Returns the learning parameters
   * @return a pointer to LearningParams objects
   */
  virtual const LearningParams* learning() const = 0;

  /**
   * @brief Returns the input dimensions of the model
   * @return : a Shape representing input dimensions
   */
  virtual Shape input_shape() const = 0;

  /**
   * @brief Returns output dimensions of the model
   * @return : a Shape representing output dimensions
   */
  virtual Shape output_shape() const = 0;

  /**
   * @brief Returns true if the input is signed and false if it is unsigned.
   * @return : a bool that indicates whether the input is signed or not
   */
  virtual bool input_signed() const = 0;

  /**
   * @brief Returns the number of layers in the model
   */
  virtual size_t get_layer_count() const = 0;

  /**
   * @brief Returns the number of MACs for the model.
   */
  virtual uint64_t get_macs() const = 0;

  /**
   * @brief Returns the hardware IP version required to map the model
   */
  virtual IpVersion get_ip_version() const = 0;

  /**
   * @brief Retrieve the Model layer sequences
   * @return a vector of const Sequence pointers
   */
  virtual const std::vector<SequencePtr>& sequences() = 0;

  /**
   * @brief Retrieve the Device the Model is mapped to
   * @return a Device pointer or nullptr
   */
  virtual DevicePtr device() = 0;
};

}  // namespace akida
