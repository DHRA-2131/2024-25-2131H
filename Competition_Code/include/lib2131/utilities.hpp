/**
 * @file utilities.hpp
 * @author Andrew Hilton (2131H)
 * @brief File for different Utilities. Features include: average and Angle.
 * @version 0.1
 * @date 2024-05-24
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <cmath>
#include <ostream>
#include <vector>

namespace lib2131
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

template <typename T>
T distance(T x0, T y0, T x1, T y1)
{
  return sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
}

}  // namespace lib2131