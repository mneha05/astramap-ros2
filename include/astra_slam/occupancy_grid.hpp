#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include "astra_slam/types.hpp"

namespace astra_slam {

class OccupancyGrid {
 public:
  OccupancyGrid(int width, int height, double resolution,
                double origin_x, double origin_y);

  bool worldToGrid(double x, double y, int &gx, int &gy) const;
  Point2 gridToWorld(int gx, int gy) const;
  void integrateRay(const Point2 &origin, const Point2 &endpoint,
                    bool hit = true);
  void integrateScan(const Pose2 &pose,
                     const std::vector<Point2> &points,
                     const std::vector<bool> &hits = {});

  float probabilityAtWorld(double x, double y) const;
  float probabilityAtGrid(int gx, int gy) const;
  std::vector<std::int8_t> toOccupancyData() const;

  int width() const { return width_; }
  int height() const { return height_; }
  double resolution() const { return resolution_; }
  double originX() const { return origin_x_; }
  double originY() const { return origin_y_; }

 private:
  bool inBounds(int gx, int gy) const;
  int index(int gx, int gy) const;
  void addLogOdds(int gx, int gy, float delta);

  int width_;
  int height_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<float> log_odds_;
  std::vector<bool> observed_;
};

}  // namespace astra_slam
