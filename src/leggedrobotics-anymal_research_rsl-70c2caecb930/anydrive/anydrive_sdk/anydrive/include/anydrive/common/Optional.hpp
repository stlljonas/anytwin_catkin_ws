#pragma once

namespace anydrive {
namespace common {

//! Class to implement optional values.
template <typename ValueT>
class Optional {
 protected:
  //! Indicate if the value has been set.
  bool isSet_ = false;
  //! Value.
  ValueT value_;

 public:
  /*!
   * Constructor from value.
   * @param defaultValue Default value.
   */
  explicit Optional(const ValueT& defaultValue = ValueT()) : value_(defaultValue) {}

  /*!
   * Copy constructor.
   * @param other Other optional.
   */
  Optional(const Optional<ValueT>& other) : isSet_(other.isSet()), value_(other.get()) {}

  /*!
   * Destructor.
   */
  virtual ~Optional() = default;

  /*!
   * Check if an optional value is set.
   */
  bool isSet() const { return isSet_; }

  /*!
   * Set the raw value.
   * @param value Raw value.
   */
  void set(const ValueT& value) {
    value_ = value;
    isSet_ = true;
  }

  /*!
   * Assignment operator using the raw value.
   * @param value Raw value.
   */
  Optional<ValueT>& operator=(const ValueT& value) {
    set(value);
    return *this;
  }

  /*!
   * Get the raw value. Does not check if it has been set.
   * @return Raw value.
   */
  const ValueT& get() const { return value_; }

  /*!
   * Cast the value to its raw type.
   * @return Raw value.
   */
  operator ValueT() const {  // NOLINT
    return get();
  }
};

}  // namespace common
}  // namespace anydrive
