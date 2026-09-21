#include "astra_slam/occupancy_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace astra_slam {
namespace {
constexpr float kFreeUpdate = -0.42F;
constexpr float kOccupiedUpdate = 0.85F;
constexpr float kMinLogOdds = -4.0F;
constexpr float kMaxLogOdds = 4.0F;
}  // namespace

OccupancyGrid::OccupancyGrid(int width, int height, double resolution,
                             double origin_x, double origin_y)
    : width_(width),
      height_(height),
      resolution_(resolution),
      origin_x_(origin_x),
      origin_y_(origin_y),
      log_odds_(static_cast<std::size_t>(width * height), 0.0F),
      observed_(static_cast<std::size_t>(width * height), false) {
  if (width <= 0 || height <= 0 || resolution <= 0.0) {
    throw std::invalid_argument("grid dimensions and resolution must be positive");
  }
}

bool OccupancyGrid::worldToGrid(double x, double y, int &gx, int &gy) const {
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));
  return inBounds(gx, gy);
}

Point2 OccupancyGrid::gridToWorld(int gx, int gy) const {
  return {origin_x_ + (static_cast<double>(gx) + 0.5) * resolution_,
          origin_y_ + (static_cast<double>(gy) + 0.5) * resolution_};
}

void OccupancyGrid::integrateRay(const Point2 &origin, const Point2 &endpoint,
                                 bool hit) {
  int x0 = 0;
  int y0 = 0;
  int x1 = 0;
  int y1 = 0;
  if (!worldToGrid(origin.x, origin.y, x0, y0) ||
      !worldToGrid(endpoint.x, endpoint.y, x1, y1)) {
    return;
  }

  const int dx = std::abs(x1 - x0);
  const int sx = x0 < x1 ? 1 : -1;
  const int dy = -std::abs(y1 - y0);
  const int sy = y0 < y1 ? 1 : -1;
  int error = dx + dy;
  int x = x0;
  int y = y0;

  while (x != x1 || y != y1) {
    addLogOdds(x, y, kFreeUpdate);
    const int doubled = 2 * error;
    if (doubled >= dy) {
      error += dy;
      x += sx;
    }
    if (doubled <= dx) {
      error += dx;
      y += sy;
    }
  }

  addLogOdds(x1, y1, hit ? kOccupiedUpdate : kFreeUpdate);
}

void OccupancyGrid::integrateScan(const Pose2 &pose,
                                  const std::vector<Point2> &points,
                                  const std::vector<bool> &hits) {
  const Point2 origin{pose.x, pose.y};
  for (std::size_t i = 0; i < points.size(); ++i) {
    const bool hit = hits.empty() || (i < hits.size() && hits[i]);
    integrateRay(origin, transformPoint(pose, points[i]), hit);
  }
}

float OccupancyGrid::probabilityAtWorld(double x, double y) const {
  int gx = 0;
  int gy = 0;
  return worldToGrid(x, y, gx, gy) ? probabilityAtGrid(gx, gy) : 0.5F;
}

float OccupancyGrid::probabilityAtGrid(int gx, int gy) const {
  if (!inBounds(gx, gy)) return 0.5F;
  const float value = log_odds_[index(gx, gy)];
  return 1.0F / (1.0F + std::exp(-value));
}

std::vector<std::int8_t> OccupancyGrid::toOccupancyData() const {
  std::vector<std::int8_t> data(log_odds_.size(), -1);
  for (std::size_t i = 0; i < data.size(); ++i) {
    if (observed_[i]) {
      const float probability = 1.0F / (1.0F + std::exp(-log_odds_[i]));
      data[i] = static_cast<std::int8_t>(std::lround(probability * 100.0F));
    }
  }
  return data;
}

bool OccupancyGrid::inBounds(int gx, int gy) const {
  return gx >= 0 && gy >= 0 && gx < width_ && gy < height_;
}

int OccupancyGrid::index(int gx, int gy) const { return gy * width_ + gx; }

void OccupancyGrid::addLogOdds(int gx, int gy, float delta) {
  if (!inBounds(gx, gy)) return;
  const int i = index(gx, gy);
  log_odds_[i] = std::clamp(log_odds_[i] + delta, kMinLogOdds, kMaxLogOdds);
  observed_[i] = true;
}

}  // namespace astra_slam
