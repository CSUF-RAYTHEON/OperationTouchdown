#pragma once

#include <string>

namespace akida {

enum class BackendType { Software, Hardware, Hybrid };

inline std::string backend_name(BackendType backend) {
  switch (backend) {
    case BackendType::Software:
      return "Software";
    case BackendType::Hardware:
      return "Hardware";
    case BackendType::Hybrid:
      return "Hybrid";
    default:
      return "Unknown";
  }
}

}  // namespace akida
