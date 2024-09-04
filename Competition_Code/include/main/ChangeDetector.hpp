#pragma once

template <typename T>
class ChangeDetector
{
 private:  // Variables
  T value;
  bool changed;

 public:  // Constructor
  ChangeDetector() : value(), changed(false) {}

  bool check(T newValue)
  {
    changed = (newValue != value);
    value = newValue;
    return changed;
  }

 public:  // Functions
  T getValue() { return value; }
  T getChanged() { return changed; }
};