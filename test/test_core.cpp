#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "astra_slam/occupancy_grid.hpp"
#include "astra_slam/scan_matcher.hpp"

namespace astra_slam {

TEST(OccupancyGrid, IntegratesFreeSpaceAndHit) {
  OccupancyGrid grid(100, 100, 0.1, -5.0, -5.0);
  grid.integrateRay({0.0, 0.0}, {2.0, 0.0}, true);
  EXPECT_LT(grid.probabilityAtWorld(1.0, 0.0), 0.5F);
  EXPECT_GT(grid.probabilityAtWorld(2.0, 0.0), 0.5F);
}

TEST(OccupancyGrid, ExportsRosCompatibleUnknownCells) {
  OccupancyGrid grid(10, 12, 0.1, -0.5, -0.6);
  const auto data = grid.toOccupancyData();
  ASSERT_EQ(data.size(), 120U);
  EXPECT_EQ(data.front(), -1);
}

TEST(CorrelativeScanMatcher, PullsPredictionTowardOccupiedEndpoints) {
  OccupancyGrid grid(200, 200, 0.05, -5.0, -5.0);
  const std::vector<Point2> scan{{2.0, 0.0}, {2.0, 0.2}, {2.0, -0.2}};
  for (int i = 0; i < 6; ++i) grid.integrateScan({0.0, 0.0, 0.0}, scan);
  CorrelativeScanMatcher matcher(0.20, 0.0, 0.05, 0.01);
  const auto result = matcher.match(grid, {0.15, 0.0, 0.0}, scan);
  EXPECT_NEAR(result.pose.x, 0.0, 0.06);
  EXPECT_GT(result.score, 0.5);
}

}  // namespace astra_slam
