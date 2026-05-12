#pragma once

#include "akida/device.h"
#include "akida/hardware_ident.h"

namespace akida {

class MapConstraints;

using MapConstraintsPtr = std::shared_ptr<MapConstraints>;

class AKIDASHAREDLIB_EXPORT MapConstraints {
 public:
  /**
   * @brief Builds a MapConstraints from a Device.
   *
   * @param device : the device from obtain mapping constraints
   */
  explicit MapConstraints(const akida::DevicePtr& device);
  MapConstraints(const MapConstraints&) = default;
  MapConstraints(MapConstraints&&) = default;
  MapConstraints& operator=(const MapConstraints&) = default;
  MapConstraints& operator=(MapConstraints&&) = default;
  virtual ~MapConstraints() = default;

  /**
   * @brief Callback next layer to map
   *
   * This callback allows the user, for example, to specify NPs number by layer.
   * To do this, user overrides the function and sets the desired split values
   * for each layer.
   *
   * @param layer_name : the layer name
   */
  virtual void next_layer_handler(const std::string& layer_name) const;

  /**
   * @brief Select a set of Neural Processors (NP)
   *
   * This allows to select from a predefined list a specified number of NPs.
   *
   * @return the list of NPs
   */
  virtual akida::hw::IdentVector select_nps(
      const akida::hw::IdentVector& source_nps, size_t num_nps,
      akida::hw::Type type) const;

  // constraints form user
  /**
   * @brief Get the maximum width allowed for a CNP
   */
  uint32_t cnp_max_width() const;
  /**
   * @brief Get the maximum height allowed for a CNP
   */
  uint32_t cnp_max_height() const;
  /**
   * @brief Get the maximum number of filters allowed for a CNP
   */
  uint32_t cnp_max_filters() const;

  /**
   * @brief Set the maximum width allowed for a CNP
   *
   * @param val : the new maximum width
   */
  void set_cnp_max_width(uint32_t val);
  /**
   * @brief Set the maximum height allowed for a CNP
   *
   * @param val : the new maximum height
   */
  void set_cnp_max_height(uint32_t val);
  /**
   * @brief Set the maximum filters allowed for a CNP
   *
   * @param val : the new maximum filters
   */
  void set_cnp_max_filters(uint32_t val);

  // constraints from HW
  /**
   * @brief Get the default maximum width allowed for a CNP
   */
  uint32_t cnp_max_width_default() const;
  /**
   * @brief Get the default maximum height allowed for a CNP
   */
  uint32_t cnp_max_height_default() const;
  /**
   * @brief Get the default maximum number of filters allowed for a CNP
   */
  uint32_t cnp_max_filters_default() const;
  /**
   * @brief Set the HWPR NP loopitself mode if supported
   */
  void set_hwpr_loopitself(bool enable);
  /**
   * @brief Get the HWPR NP loopitself mode
   */
  bool hwpr_loopitself() const;

  /**
   * @brief Return the device used
   */
  akida::DevicePtr device() const;

 private:
  akida::DevicePtr device_;
  uint32_t cnp_max_width_default_;
  uint32_t cnp_max_height_default_;
  uint32_t cnp_max_filters_default_;
  uint32_t cnp_max_width_;
  uint32_t cnp_max_height_;
  uint32_t cnp_max_filters_;
  bool hwpr_loopitself_enable_;
  IpVersion ip_version_;
};
}  // namespace akida
