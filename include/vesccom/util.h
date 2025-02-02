#ifndef VESCCOM_INCLUDE_VESCCOM_UTIL_H_
#define VESCCOM_INCLUDE_VESCCOM_UTIL_H_

#include <cmath>

// Normalizes `angle` to [0, 360).
inline double norm_0i_360e(double angle) {
  angle = std::fmod(angle, 360);
  if (angle < 0) angle += 360;
  return angle;
}

#endif
