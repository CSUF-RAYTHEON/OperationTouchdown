#pragma once

#include <map>
#include <memory>

#include "akida/backend_type.h"

#include "infra/exports.h"

namespace akida {

class Backend;

using BackendPtr = std::shared_ptr<Backend>;

// Raises an exception if the key is not found, allowing to dereference
// the result of this function safely
AKIDASHAREDLIB_EXPORT BackendPtr get_backend(BackendType type);

// Checks if a given backend type is available
AKIDASHAREDLIB_EXPORT bool has_backend(BackendType type);

// Return the full list of available backends
AKIDASHAREDLIB_EXPORT const std::map<BackendType, BackendPtr>& get_backends();

}  // namespace akida
