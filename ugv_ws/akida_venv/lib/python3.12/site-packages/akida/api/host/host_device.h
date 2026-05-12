#pragma once

#include <memory>

#include "akida/device.h"
#include "akida/hardware_device.h"
#include "akida/mesh.h"

namespace akida {

class HostDevice;

using HostDevicePtr = std::shared_ptr<HostDevice>;
using HostDeviceConstPtr = std::shared_ptr<const HostDevice>;

class AKIDASHAREDLIB_EXPORT HostDevice : public Device {
 public:
  explicit HostDevice(HardwareDevicePtr hardware) : hardware_(hardware) {
    mesh_ = Mesh::discover(hardware_.get());
    check_hw_and_ip_version_compatibility(mesh_->version);
  }

  HwVersion version() const override { return hardware_->version(); };

  const char* desc() const override { return hardware_->desc(); }

  const Mesh& mesh() const override { return *mesh_.get(); }

  HardwareDevice* hardware() const override { return hardware_.get(); }

 private:
  HardwareDevicePtr hardware_;
  std::unique_ptr<Mesh> mesh_;
};

}  // namespace akida
