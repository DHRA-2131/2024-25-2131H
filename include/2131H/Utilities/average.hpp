/**
 * @file average.hpp
 * @author Andrew Hilton (2131H)
 * @brief Average Function
 * @version 0.1
 * @date 2024-10-03
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <vector>

namespace Utilities
{

/**
 * @brief Averages a vector of values
 *
 * @tparam T Typename of vector / values
 * @param array Array of values
 * @return T Average of array
 */
template <typename T>
T avg(std::vector<T> array)
{
  T sum = 0;
  for (auto i : array) { sum += i; }
  return sum / array.size();
}
}  // namespace Utilities