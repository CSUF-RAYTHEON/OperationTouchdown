#pragma once
#include <memory>
#include "host/akd1500_spiflash_controller.h"

namespace akida {

class FlashAccessDriver {
 public:
  virtual ~FlashAccessDriver() = default;
  virtual std::shared_ptr<SpiFlashController> get_spi_flash_controller() = 0;
};

}  // namespace akida
