#pragma once

/**
 * @brief ChangeDetector detects changes of any type
 *
 * @tparam T typename of target value
 */
template <typename T>
class ChangeDetector
{
 private:        // Variables
  T value;       // Current Value
  bool changed;  // Did the value change last check?

 public:  // Constructor
  /**
   * @brief Construct a new Change Detector
   *
   */
  ChangeDetector(T initalValue = 0) : value(), changed(false) {}

  /**
   * @brief Returns whether a value has changed
   *
   * @param newValue New Value to check against last loop
   * @return true Value Changed
   * @return false Value didn't change
   */
  bool check(T newValue)
  {
    changed = (newValue != value);  // is the new value different (not equal)
    value = newValue;               // update value
    return changed;
  }

 public:  // Functions
  /**
   * @brief Get the Value stored in ChangeDetector from last loop
   *
   * @return T value
   */
  T getValue() { return value; }

  /**
   * @brief Get if the value changed last loop
   *
   * @return bool changed
   */
  bool getChanged() { return changed; }
};