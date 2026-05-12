#pragma once

#include <cstdlib>

namespace akida {

inline char* getenv(const char* name) {
#if defined(__clang__) && defined(_WIN32)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#endif
  return std::getenv(name);
#if defined(__clang__) && defined(_WIN32)
#pragma clang diagnostic pop
#endif
}
}  // namespace akida
