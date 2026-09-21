#pragma once

#include <vector>

#include "astra_slam/occupancy_grid.hpp"
#include "astra_slam/types.hpp"

namespace astra_slam {

struct MatchResult {
  Pose2 pose;
  double score{0.0};
};

class CorrelativeScanMatcher {
 public:
  CorrelativeScanMatcher(double translation_window = 0.20,
                         double rotation_window = 0.10,
                         double translation_step = 0.04,
                         double rotation_step = 0.025);

  MatchResult match(const OccupancyGrid &grid, const Pose2 &prediction,
                    const std::vector<Point2> &scan_points) const;

 private:
  double scorePose(const OccupancyGrid &grid, const Pose2 &pose,
                   const std::vector<Point2> &scan_points) const;

  double translation_window_;
  double rotation_window_;
  double translation_step_;
  double rotation_step_;
};

}  // namespace astra_slam
