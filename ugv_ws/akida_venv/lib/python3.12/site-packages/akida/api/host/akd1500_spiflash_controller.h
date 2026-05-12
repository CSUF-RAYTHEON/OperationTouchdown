#pragma once
#include <stdint.h>

class SpiFlashController {
 public:
  virtual ~SpiFlashController() = default;

  uint16_t get_id() const;
};
