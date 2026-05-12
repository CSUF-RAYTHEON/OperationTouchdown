#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "infra/exports.h"

namespace akida {

/**
 * @enum LearningType
 * @brief The layer type
 */
enum class LearningType { AkidaUnsupervised };

class LearningParams;
using LearningParamsPtr = std::shared_ptr<LearningParams>;

/**
 * @class LearningParams
 * @brief Generic parameters that can be used for several learnings
 */
class AKIDASHAREDLIB_EXPORT LearningParams {
 public:
  // Value is a basic type large enough to hold the data for every type in a
  // lossless way.
  using Value = double;
  using KeyType = const std::string;
  using Dict = std::map<KeyType, Value>;

  virtual ~LearningParams() = default;

  /**
   * @brief Simplified static cast to Value
   * @param value: Value to be casted
   * @return casted value, ready to be inserted
   */
  template<typename T>
  static constexpr Value as_value(T value) {
    return static_cast<Value>(value);
  }

  /**
   * @brief Create a LearningParams object
   * @param learning_type: learning type for these parameters
   * @param entries: a map of key-entries
   * @return a LearningParams object
   */
  static LearningParamsPtr create(LearningType type, const Dict& entries);

  /**
   * @brief Copy a LearningParams object
   * @param src: source object
   * @return A copy of the source object
   */
  static LearningParamsPtr clone(const LearningParams* src);

  /**
   * @brief Reads a value from LearningParams.
   * @param key: identifier for the value
   * @return Value associated with the key
   */
  virtual Value get(KeyType const& key) const = 0;

  /**
   * @brief Get all the keys in the LearningParams instance.
   * @return Vector of keys
   */
  virtual std::vector<std::string> keys() const = 0;

  /**
   * @brief compare two LearningParams objects.
   * @param other: object to compare with
   * @return true if values and learning type are the same
   */
  virtual bool operator==(const LearningParams& other) const = 0;

  /**
   * @brief Get the learning type
   * @return the learning type
   */
  virtual LearningType learning_type() const = 0;
};

}  // namespace akida