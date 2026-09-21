#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "astra_slam/types.hpp"

namespace astra_slam {
namespace {

struct Vertex {
  float x;
  float y;
  float r;
  float g;
  float b;
};

GLuint compileShader(GLenum type, const char *source) {
  const GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, nullptr);
  glCompileShader(shader);
  GLint success = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (success != GL_TRUE) {
    char log[1024];
    glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
    throw std::runtime_error(std::string("shader compilation failed: ") + log);
  }
  return shader;
}

GLuint createProgram() {
  constexpr const char *vertex_source = R"(
    #version 330 core
    layout(location = 0) in vec2 position;
    layout(location = 1) in vec3 color;
    out vec3 vertexColor;
    void main() {
      gl_Position = vec4(position, 0.0, 1.0);
      vertexColor = color;
    })";
  constexpr const char *fragment_source = R"(
    #version 330 core
    in vec3 vertexColor;
    out vec4 pixel;
    void main() { pixel = vec4(vertexColor, 1.0); })";
  const GLuint vertex = compileShader(GL_VERTEX_SHADER, vertex_source);
  const GLuint fragment = compileShader(GL_FRAGMENT_SHADER, fragment_source);
  const GLuint program = glCreateProgram();
  glAttachShader(program, vertex);
  glAttachShader(program, fragment);
  glLinkProgram(program);
  glDeleteShader(vertex);
  glDeleteShader(fragment);
  GLint success = GL_FALSE;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success != GL_TRUE) throw std::runtime_error("shader link failed");
  return program;
}
}  // namespace

class OpenGlViewerNode : public rclcpp::Node {
 public:
  OpenGlViewerNode() : Node("astra_opengl_viewer") {
    using std::placeholders::_1;
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 1, std::bind(&OpenGlViewerNode::onMap, this, _1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(),
        std::bind(&OpenGlViewerNode::onScan, this, _1));
    radar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/radar/detections", rclcpp::SensorDataQoS(),
        std::bind(&OpenGlViewerNode::onRadar, this, _1));
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/slam/pose", 10, std::bind(&OpenGlViewerNode::onPose, this, _1));
  }

  void run() {
    if (!glfwInit()) throw std::runtime_error("GLFW initialization failed");
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow *window = glfwCreateWindow(1280, 820,
                                          "AstraMap | ROS 2 Sensor-Fusion SLAM",
                                          nullptr, nullptr);
    if (!window) {
      glfwTerminate();
      throw std::runtime_error("OpenGL window creation failed");
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) throw std::runtime_error("GLEW initialization failed");

    const GLuint program = createProgram();
    GLuint vao = 0;
    GLuint vbo = 0;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), nullptr);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          reinterpret_cast<void *>(2 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glEnable(GL_PROGRAM_POINT_SIZE);

    while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
      rclcpp::spin_some(shared_from_this());
      int width = 0;
      int height = 0;
      glfwGetFramebufferSize(window, &width, &height);
      glViewport(0, 0, width, height);
      glClearColor(0.018F, 0.027F, 0.055F, 1.0F);
      glClear(GL_COLOR_BUFFER_BIT);
      glUseProgram(program);
      glBindVertexArray(vao);
      draw(vbo, map_vertices_, GL_POINTS, 3.0F);
      draw(vbo, scan_vertices_, GL_POINTS, 4.0F);
      draw(vbo, radar_vertices_, GL_POINTS, 13.0F);
      draw(vbo, robot_vertices_, GL_TRIANGLES, 1.0F);
      glfwSwapBuffers(window);
      glfwPollEvents();
    }
    glDeleteBuffers(1, &vbo);
    glDeleteVertexArrays(1, &vao);
    glDeleteProgram(program);
    glfwDestroyWindow(window);
    glfwTerminate();
  }

 private:
  Vertex vertex(double x, double y, float r, float g, float b) const {
    const double width = std::max(1e-6, world_max_x_ - world_min_x_);
    const double height = std::max(1e-6, world_max_y_ - world_min_y_);
    return {static_cast<float>(2.0 * (x - world_min_x_) / width - 1.0),
            static_cast<float>(2.0 * (y - world_min_y_) / height - 1.0),
            r, g, b};
  }

  static void draw(GLuint vbo, const std::vector<Vertex> &vertices,
                   GLenum primitive, float point_size) {
    if (vertices.empty()) return;
    glPointSize(point_size);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 static_cast<GLsizeiptr>(vertices.size() * sizeof(Vertex)),
                 vertices.data(), GL_DYNAMIC_DRAW);
    glDrawArrays(primitive, 0, static_cast<GLsizei>(vertices.size()));
  }

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    world_min_x_ = map->info.origin.position.x;
    world_min_y_ = map->info.origin.position.y;
    world_max_x_ = world_min_x_ + map->info.width * map->info.resolution;
    world_max_y_ = world_min_y_ + map->info.height * map->info.resolution;
    map_vertices_.clear();
    for (std::uint32_t y = 0; y < map->info.height; y += 2) {
      for (std::uint32_t x = 0; x < map->info.width; x += 2) {
        const auto value = map->data[y * map->info.width + x];
        if (value < 55) continue;
        const float confidence = std::clamp(value / 100.0F, 0.0F, 1.0F);
        map_vertices_.push_back(vertex(
            world_min_x_ + (x + 0.5) * map->info.resolution,
            world_min_y_ + (y + 0.5) * map->info.resolution,
            0.25F + 0.45F * confidence, 0.42F + 0.45F * confidence, 1.0F));
      }
    }
  }

  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    current_pose_.x = pose->pose.position.x;
    current_pose_.y = pose->pose.position.y;
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose->pose.orientation, quaternion);
    double roll = 0.0;
    double pitch = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, current_pose_.yaw);
    have_pose_ = true;

    robot_vertices_.clear();
    const Point2 nose = transformPoint(current_pose_, {0.38, 0.0});
    const Point2 left = transformPoint(current_pose_, {-0.25, 0.22});
    const Point2 right = transformPoint(current_pose_, {-0.25, -0.22});
    robot_vertices_.push_back(vertex(nose.x, nose.y, 0.2F, 1.0F, 0.72F));
    robot_vertices_.push_back(vertex(left.x, left.y, 0.2F, 0.8F, 1.0F));
    robot_vertices_.push_back(vertex(right.x, right.y, 0.2F, 0.8F, 1.0F));
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (!have_pose_) return;
    scan_vertices_.clear();
    for (std::size_t i = 0; i < scan->ranges.size(); i += 2) {
      const float range = scan->ranges[i];
      if (!std::isfinite(range) || range >= scan->range_max) continue;
      const double angle = scan->angle_min + i * scan->angle_increment;
      const Point2 world = transformPoint(
          current_pose_, {range * std::cos(angle), range * std::sin(angle)});
      scan_vertices_.push_back(vertex(world.x, world.y, 0.15F, 0.95F, 0.92F));
    }
  }

  void onRadar(const sensor_msgs::msg::PointCloud2::SharedPtr cloud) {
    if (!have_pose_) return;
    radar_vertices_.clear();
    sensor_msgs::PointCloud2ConstIterator<float> x(*cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(*cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> velocity(*cloud, "velocity");
    for (; x != x.end(); ++x, ++y, ++velocity) {
      const Point2 world = transformPoint(current_pose_, {*x, *y});
      const float speed = std::min(1.0F, std::abs(*velocity));
      radar_vertices_.push_back(vertex(world.x, world.y, 1.0F, 0.22F + 0.4F * speed,
                                        0.25F));
    }
  }

  double world_min_x_{-10.0};
  double world_max_x_{10.0};
  double world_min_y_{-7.5};
  double world_max_y_{7.5};
  Pose2 current_pose_;
  bool have_pose_{false};
  std::vector<Vertex> map_vertices_;
  std::vector<Vertex> scan_vertices_;
  std::vector<Vertex> radar_vertices_;
  std::vector<Vertex> robot_vertices_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr radar_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
};

}  // namespace astra_slam

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<astra_slam::OpenGlViewerNode>();
  try {
    node->run();
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
