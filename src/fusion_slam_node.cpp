#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "astra_slam/occupancy_grid.hpp"
#include "astra_slam/scan_matcher.hpp"

namespace astra_slam {

class FusionSlamNode : public rclcpp::Node {
 public:
  FusionSlamNode()
      : Node("astra_fusion_slam"),
        map_(declare_parameter("map_width", 400),
             declare_parameter("map_height", 300),
             declare_parameter("map_resolution", 0.05),
             declare_parameter("map_origin_x", -10.0),
             declare_parameter("map_origin_y", -7.5)),
        matcher_(declare_parameter("translation_window", 0.16),
                 declare_parameter("rotation_window", 0.08), 0.04, 0.02) {
    using std::placeholders::_1;
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&FusionSlamNode::onScan, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/wheel/odometry", 20, std::bind(&FusionSlamNode::onOdometry, this, _1));
    radar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/radar/detections", rclcpp::SensorDataQoS(),
        std::bind(&FusionSlamNode::onRadar, this, _1));
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/slam/pose", 10);
    dynamic_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fusion/dynamic_obstacles", 10);
    RCLCPP_INFO(get_logger(), "LiDAR SLAM + radar fusion front end ready");
  }

 private:
  static double yawFromQuaternion(const geometry_msgs::msg::Quaternion &message) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(message, quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);
    return yaw;
  }

  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr message) {
    odom_pose_ = {message->pose.pose.position.x, message->pose.pose.position.y,
                  yawFromQuaternion(message->pose.pose.orientation)};
    have_odometry_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!have_odometry_) return;
    std::vector<Point2> points;
    std::vector<bool> hits;
    points.reserve(scan->ranges.size());
    hits.reserve(scan->ranges.size());
    for (std::size_t i = 0; i < scan->ranges.size(); ++i) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range < scan->range_min) continue;
      const double clipped = std::min<double>(range, scan->range_max);
      const double angle = scan->angle_min + i * scan->angle_increment;
      points.push_back({clipped * std::cos(angle), clipped * std::sin(angle)});
      hits.push_back(range < scan->range_max * 0.995F);
    }

    if (!initialized_) {
      slam_pose_ = odom_pose_;
      initialized_ = true;
    } else {
      Pose2 prediction{
          slam_pose_.x + (odom_pose_.x - previous_odom_pose_.x),
          slam_pose_.y + (odom_pose_.y - previous_odom_pose_.y),
          normalizeAngle(slam_pose_.yaw +
                         normalizeAngle(odom_pose_.yaw - previous_odom_pose_.yaw))};
      slam_pose_ = matcher_.match(map_, prediction, points).pose;
    }
    previous_odom_pose_ = odom_pose_;

    map_.integrateScan(slam_pose_, points, hits);
    publishMap(scan->header.stamp);
    publishPose(scan->header.stamp);
  }

  void onRadar(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!initialized_) return;
    visualization_msgs::msg::MarkerArray markers;
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> velocity(*cloud, "velocity");
    int id = 0;
    for (; x != x.end(); ++x, ++y, ++velocity, ++id) {
      const Point2 world = transformPoint(slam_pose_, {*x, *y});
      visualization_msgs::msg::Marker marker;
      marker.header.stamp = cloud->header.stamp;
      marker.header.frame_id = "map";
      marker.ns = "radar_tracks";
      marker.id = id;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = world.x;
      marker.pose.position.y = world.y;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = marker.scale.z = 0.34;
      const float speed = std::abs(*velocity);
      marker.color.r = std::min(1.0F, 0.25F + speed);
      marker.color.g = 0.35F;
      marker.color.b = 1.0F - std::min(0.8F, speed);
      marker.color.a = 0.92F;
      marker.lifetime = rclcpp::Duration::from_seconds(0.15);
      markers.markers.push_back(marker);
    }
    dynamic_pub_->publish(markers);
  }

  void publishMap(const builtin_interfaces::msg::Time &stamp) {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.info.resolution = static_cast<float>(map_.resolution());
    message.info.width = static_cast<std::uint32_t>(map_.width());
    message.info.height = static_cast<std::uint32_t>(map_.height());
    message.info.origin.position.x = map_.originX();
    message.info.origin.position.y = map_.originY();
    message.info.origin.orientation.w = 1.0;
    message.data = map_.toOccupancyData();
    map_pub_->publish(message);
  }

  void publishPose(const builtin_interfaces::msg::Time &stamp) {
    geometry_msgs::msg::PoseStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = "map";
    message.pose.position.x = slam_pose_.x;
    message.pose.position.y = slam_pose_.y;
    tf2::Quaternion orientation;
    orientation.setRPY(0.0, 0.0, slam_pose_.yaw);
    message.pose.orientation = tf2::toMsg(orientation);
    pose_pub_->publish(message);
  }

  OccupancyGrid map_;
  CorrelativeScanMatcher matcher_;
  Pose2 odom_pose_;
  Pose2 previous_odom_pose_;
  Pose2 slam_pose_;
  bool have_odometry_{false};
  bool initialized_{false};
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr dynamic_pub_;
};

}  // namespace astra_slam

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<astra_slam::FusionSlamNode>());
  rclcpp::shutdown();
  return 0;
}
