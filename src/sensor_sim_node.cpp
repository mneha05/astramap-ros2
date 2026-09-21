#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

#include "astra_slam/types.hpp"

namespace astra_slam {
namespace {
constexpr double kPi = 3.14159265358979323846;

struct Segment {
  Point2 a;
  Point2 b;
};

struct MovingTarget {
  Point2 position;
  Point2 velocity;
};

double raySegmentDistance(const Point2 &origin, double angle,
                          const Segment &segment) {
  const Point2 direction{std::cos(angle), std::sin(angle)};
  const Point2 edge{segment.b.x - segment.a.x, segment.b.y - segment.a.y};
  const Point2 offset{segment.a.x - origin.x, segment.a.y - origin.y};
  const double cross = direction.x * edge.y - direction.y * edge.x;
  if (std::abs(cross) < 1e-9) return std::numeric_limits<double>::infinity();
  const double distance = (offset.x * edge.y - offset.y * edge.x) / cross;
  const double segment_t = (offset.x * direction.y - offset.y * direction.x) / cross;
  if (distance >= 0.0 && segment_t >= 0.0 && segment_t <= 1.0) return distance;
  return std::numeric_limits<double>::infinity();
}

std::array<Segment, 4> box(double x0, double y0, double x1, double y1) {
  return {{{{x0, y0}, {x1, y0}},
           {{x1, y0}, {x1, y1}},
           {{x1, y1}, {x0, y1}},
           {{x0, y1}, {x0, y0}}}};
}
}  // namespace

class SensorSimNode : public rclcpp::Node {
 public:
  SensorSimNode() : Node("astra_sensor_sim"), rng_(42), noise_(0.0, 0.012) {
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    radar_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/radar/detections", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/wheel/odometry", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    addBox(-9.0, -6.0, 9.0, 6.0);
    addBox(-2.5, -1.2, -0.7, 1.8);
    addBox(2.0, -4.2, 3.3, -1.0);
    addBox(3.7, 1.5, 6.2, 3.6);
    walls_.push_back({{-7.0, 3.8}, {-2.0, 3.8}});

    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&SensorSimNode::tick, this));
    RCLCPP_INFO(get_logger(), "Synthetic LiDAR/radar scene running at 20 Hz");
  }

 private:
  void addBox(double x0, double y0, double x1, double y1) {
    const auto segments = box(x0, y0, x1, y1);
    walls_.insert(walls_.end(), segments.begin(), segments.end());
  }

  Pose2 groundTruthPose() const {
    const double phase = 0.12 * time_;
    const double x = 5.1 * std::cos(phase);
    const double y = 3.3 * std::sin(phase);
    const double dx = -0.612 * std::sin(phase);
    const double dy = 0.396 * std::cos(phase);
    return {x, y, std::atan2(dy, dx)};
  }

  std::vector<MovingTarget> movingTargets() const {
    return {{{1.5 + 1.6 * std::sin(0.35 * time_),
              0.4 + 1.1 * std::cos(0.35 * time_)},
             {0.56 * std::cos(0.35 * time_), -0.385 * std::sin(0.35 * time_)}},
            {{-4.0 + 0.9 * std::cos(0.55 * time_),
              -2.7 + 0.7 * std::sin(0.55 * time_)},
             {-0.495 * std::sin(0.55 * time_), 0.385 * std::cos(0.55 * time_)}}};
  }

  void tick() {
    time_ += 0.05;
    const auto stamp = now();
    const Pose2 truth = groundTruthPose();
    publishOdometry(truth, stamp);
    publishScan(truth, stamp);
    publishRadar(truth, stamp);
  }

  void publishOdometry(const Pose2 &truth, const rclcpp::Time &stamp) {
    nav_msgs::msg::Odometry message;
    message.header.stamp = stamp;
    message.header.frame_id = "odom";
    message.child_frame_id = "base_link";
    message.pose.pose.position.x = truth.x + 0.035 * std::sin(0.07 * time_);
    message.pose.pose.position.y = truth.y + 0.025 * std::cos(0.09 * time_);
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, truth.yaw + 0.008 * std::sin(0.11 * time_));
    message.pose.pose.orientation.x = orientation.x();
    message.pose.pose.orientation.y = orientation.y();
    message.pose.pose.orientation.z = orientation.z();
    message.pose.pose.orientation.w = orientation.w();
    odom_pub_->publish(message);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = message.header;
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = message.pose.pose.position.x;
    transform.transform.translation.y = message.pose.pose.position.y;
    transform.transform.rotation = message.pose.pose.orientation;
    tf_broadcaster_->sendTransform(transform);
  }

  void publishScan(const Pose2 &pose, const rclcpp::Time &stamp) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = stamp;
    scan.header.frame_id = "base_link";
    scan.angle_min = static_cast<float>(-kPi);
    scan.angle_max = static_cast<float>(kPi);
    scan.angle_increment = static_cast<float>(kPi / 180.0);
    scan.range_min = 0.12F;
    scan.range_max = 14.0F;
    scan.scan_time = 0.05F;
    scan.time_increment = scan.scan_time / 360.0F;
    scan.ranges.resize(360);

    for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
      const double local_angle = scan.angle_min + i * scan.angle_increment;
      const double world_angle = pose.yaw + local_angle;
      double distance = scan.range_max;
      for (const Segment &wall : walls_) {
        distance = std::min(distance, raySegmentDistance({pose.x, pose.y}, world_angle, wall));
      }
      scan.ranges[i] = static_cast<float>(std::clamp(distance + noise_(rng_),
                                                     static_cast<double>(scan.range_min),
                                                     static_cast<double>(scan.range_max)));
    }
    scan_pub_->publish(scan);
  }

  void publishRadar(const Pose2 &pose, const rclcpp::Time &stamp) {
    const auto targets = movingTargets();
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(targets.size());
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2Fields(5,
        "x", 1, sensor_msgs::msg::PointField::FLOAT32,
        "y", 1, sensor_msgs::msg::PointField::FLOAT32,
        "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
        "velocity", 1, sensor_msgs::msg::PointField::FLOAT32);
    modifier.resize(targets.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
    sensor_msgs::PointCloud2Iterator<float> velocity(cloud, "velocity");
    const double c = std::cos(-pose.yaw);
    const double s = std::sin(-pose.yaw);
    for (const auto &target : targets) {
      const double dx = target.position.x - pose.x;
      const double dy = target.position.y - pose.y;
      const double local_x = c * dx - s * dy;
      const double local_y = s * dx + c * dy;
      const double range = std::hypot(local_x, local_y);
      const double radial_velocity = range > 1e-6
          ? (target.velocity.x * dx + target.velocity.y * dy) / range : 0.0;
      *x = static_cast<float>(local_x);
      *y = static_cast<float>(local_y);
      *z = 0.0F;
      *intensity = static_cast<float>(1.0 / (1.0 + 0.05 * range * range));
      *velocity = static_cast<float>(radial_velocity);
      ++x; ++y; ++z; ++intensity; ++velocity;
    }
    radar_pub_->publish(cloud);
  }

  double time_{0.0};
  std::vector<Segment> walls_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radar_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace astra_slam

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_slam::SensorSimNode>());
  rclcpp::shutdown();
  return 0;
}
