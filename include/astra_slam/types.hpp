#pragma once

#include <cmath>

namespace astra_slam {

struct Point2 {
  double x{0.0};
  double y{0.0};
};

struct Pose2 {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Point2 transformPoint(const Pose2 &pose, const Point2 &point) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {pose.x + c * point.x - s * point.y,
          pose.y + s * point.x + c * point.y};
}

inline double normalizeAngle(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) angle -= 2.0 * kPi;
  while (angle < -kPi) angle += 2.0 * kPi;
  return angle;
}

}  // namespace astra_slam
