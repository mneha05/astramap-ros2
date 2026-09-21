#include "astra_slam/scan_matcher.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace astra_slam {

CorrelativeScanMatcher::CorrelativeScanMatcher(double translation_window,
                                               double rotation_window,
                                               double translation_step,
                                               double rotation_step)
    : translation_window_(translation_window),
      rotation_window_(rotation_window),
      translation_step_(translation_step),
      rotation_step_(rotation_step) {}

MatchResult CorrelativeScanMatcher::match(
    const OccupancyGrid &grid, const Pose2 &prediction,
    const std::vector<Point2> &scan_points) const {
  MatchResult best{prediction, -std::numeric_limits<double>::infinity()};
  for (double dx = -translation_window_; dx <= translation_window_ + 1e-9;
       dx += translation_step_) {
    for (double dy = -translation_window_; dy <= translation_window_ + 1e-9;
         dy += translation_step_) {
      for (double dyaw = -rotation_window_; dyaw <= rotation_window_ + 1e-9;
           dyaw += rotation_step_) {
        Pose2 candidate{prediction.x + dx, prediction.y + dy,
                        normalizeAngle(prediction.yaw + dyaw)};
        const double score = scorePose(grid, candidate, scan_points);
        if (score > best.score) best = {candidate, score};
      }
    }
  }
  return best;
}

double CorrelativeScanMatcher::scorePose(
    const OccupancyGrid &grid, const Pose2 &pose,
    const std::vector<Point2> &scan_points) const {
  if (scan_points.empty()) return 0.0;
  double score = 0.0;
  std::size_t samples = 0;
  const std::size_t stride = std::max<std::size_t>(1, scan_points.size() / 90);
  for (std::size_t i = 0; i < scan_points.size(); i += stride) {
    const Point2 world = transformPoint(pose, scan_points[i]);
    const double probability = grid.probabilityAtWorld(world.x, world.y);
    score += probability;
    ++samples;
  }
  return samples == 0 ? 0.0 : score / static_cast<double>(samples);
}

}  // namespace astra_slam
