#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "akida/dense.h"
#include "infra/exports.h"

namespace akida {

/**
 * class Variables
 * @brief Public interface to access and use layer variables.
 */
class AKIDASHAREDLIB_EXPORT Variables {
 public:
  virtual ~Variables() {}

  /**
   * @brief Returns the names of available variables.
   */
  virtual std::vector<std::string> names() const = 0;

  /**
   * @brief Sets a variable.
   * @param name   : name of the variable to set
   * @param tensor : values to set as a Tensor shared pointer
   */
  virtual void set(const std::string& name, DenseConstPtr tensor) = 0;

  /**
   * @brief Gets a variable.
   * @param name : name of the variable to get
   * @return the requested variable
   * @note The returned variable is read-only, to apply changes to it one must
   * use the set method
   */
  virtual DenseConstPtr get(const std::string& name) const = 0;

  /**
   * @brief Check if variable exists.
   * @param name : name of the variable to check
   * @return boolean asserting the existence or not of the variable
   */
  virtual bool has(const std::string& name) const = 0;
};

/**
 * @brief A shared pointer to a Variables object
 */
using VariablesPtr = std::shared_ptr<Variables>;

}  // namespace akida
