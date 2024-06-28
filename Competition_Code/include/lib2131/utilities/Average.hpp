#pragma once
#include <vector>

namespace lib2131::utilities
{
/**
 * @brief Averages all values in a Vector
 *
 * @tparam typename T
 * @param B Vector
 * @return T Average
 */
template <typename T>
T average(std::vector<T> B)
{
  T Total(0);  // Total
  for (int i = 0; i < B.size(); i++)
  {
    Total += B[i];  // Add each Element
  }
  return Total / B.size();  // Divide by total Elements
}
}  // namespace lib2131::utilities