#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "slam_scan_match.hpp"

using robot_slam::Beam;
using robot_slam::normalize_angle;
using robot_slam::Pose2D;
using robot_slam::ScanMatchConfig;
using robot_slam::ScanMatcher;

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion quaternion_from_yaw(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

using Matrix3 = std::array<double, 9>;
using Vector3 = std::array<double, 3>;

Matrix3 identity_matrix()
{
  return Matrix3{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
}

Matrix3 transpose(const Matrix3 & matrix)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row * 3 + column] = matrix[column * 3 + row];
    }
  }
  return result;
}

Matrix3 multiply(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      for (std::size_t inner = 0; inner < 3; ++inner) {
        result[row * 3 + column] +=
          left[row * 3 + inner] * right[inner * 3 + column];
      }
    }
  }
  return result;
}

Matrix3 add(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = left[index] + right[index];
  }
  return result;
}

Matrix3 subtract(const Matrix3 & left, const Matrix3 & right)
{
  Matrix3 result{};
  for (std::size_t index = 0; index < result.size(); ++index) {
    result[index] = left[index] - right[index];
  }
  return result;
}

Vector3 multiply(const Matrix3 & matrix, const Vector3 & vector)
{
  Vector3 result{};
  for (std::size_t row = 0; row < 3; ++row) {
    for (std::size_t column = 0; column < 3; ++column) {
      result[row] += matrix[row * 3 + column] * vector[column];
    }
  }
  return result;
}

bool inverse(const Matrix3 & matrix, Matrix3 & result)
{
  const double a = matrix[0];
  const double b = matrix[1];
  const double c = matrix[2];
  const double d = matrix[3];
  const double e = matrix[4];
  const double f = matrix[5];
  const double g = matrix[6];
  const double h = matrix[7];
  const double i = matrix[8];
  const double determinant =
    a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  if (!std::isfinite(determinant) || std::abs(determinant) < 1e-15) {
    return false;
  }

  const double scale = 1.0 / determinant;
  result = Matrix3{
    (e * i - f * h) * scale,
    (c * h - b * i) * scale,
    (b * f - c * e) * scale,
    (f * g - d * i) * scale,
    (a * i - c * g) * scale,
    (c * d - a * f) * scale,
    (d * h - e * g) * scale,
    (b * g - a * h) * scale,
    (a * e - b * d) * scale};
  return true;
}

}  // namespace

class EkfSlamNode : public rclcpp::Node
{
public:
  EkfSlamNode()
  : Node("ekf_slam")
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/ekf/map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/ekf/pose");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    resolution_ = declare_parameter<double>("resolution", 0.05);
    map_width_ = declare_parameter<int>("map_width", 400);
    map_height_ = declare_parameter<int>("map_height", 400);
    origin_x_ = declare_parameter<double>("origin_x", -10.0);
    origin_y_ = declare_parameter<double>("origin_y", -10.0);
    max_range_ = declare_parameter<double>("max_range", 7.5);
    max_beams_ = declare_parameter<int>("max_beams", 90);
    minimum_travel_distance_ =
      declare_parameter<double>("minimum_travel_distance", 0.10);
    minimum_travel_heading_ =
      declare_parameter<double>("minimum_travel_heading", 0.08);

    linear_noise_ = declare_parameter<double>("linear_motion_noise", 0.05);
    angular_noise_ = declare_parameter<double>("angular_motion_noise", 0.05);
    noise_floor_ = declare_parameter<double>("motion_noise_floor", 0.005);
    initial_position_variance_ =
      declare_parameter<double>("initial_position_variance", 0.0001);
    initial_heading_variance_ =
      declare_parameter<double>("initial_heading_variance", 0.0001);
    measurement_position_variance_ =
      declare_parameter<double>("measurement_position_variance", 0.01);
    measurement_heading_variance_ =
      declare_parameter<double>("measurement_heading_variance", 0.0064);

    scan_linear_window_ = declare_parameter<double>("scan_match_linear_window", 0.20);
    scan_angular_window_ = declare_parameter<double>("scan_match_angular_window", 0.16);
    scan_linear_step_ = declare_parameter<double>("scan_match_linear_step", 0.05);
    scan_angular_step_ = declare_parameter<double>("scan_match_angular_step", 0.04);
    minimum_scan_score_ = declare_parameter<double>("minimum_scan_score", 0.52);
    hit_log_odds_ = declare_parameter<double>("hit_log_odds", 0.85);
    miss_log_odds_ = declare_parameter<double>("miss_log_odds", -0.40);
    map_publish_interval_ = declare_parameter<double>("map_publish_interval", 1.0);

    laser_x_ = declare_parameter<double>("laser_x", -0.032);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    validate_parameters();
    scan_matcher_.configure(ScanMatchConfig{
      map_width_, map_height_, origin_x_, origin_y_, resolution_,
      scan_linear_window_, scan_angular_window_, scan_linear_step_, scan_angular_step_});
    const std::size_t cell_count =
      static_cast<std::size_t>(map_width_) * static_cast<std::size_t>(map_height_);
    map_.assign(cell_count, 0.0F);
    covariance_ = Matrix3{
      initial_position_variance_, 0.0, 0.0,
      0.0, initial_position_variance_, 0.0,
      0.0, 0.0, initial_heading_variance_};

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos);
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&EkfSlamNode::odom_callback, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&EkfSlamNode::scan_callback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(
      get_logger(), "EKF SLAM ready; publishing %s and %s in frame %s%s",
      map_topic_.c_str(), pose_topic_.c_str(), map_frame_.c_str(),
      publish_tf_ ? " with map-to-odom TF" : " without TF (comparison mode)");
  }

private:
  void validate_parameters() const
  {
    if (resolution_ <= 0.0 || map_width_ <= 0 || map_height_ <= 0) {
      throw std::invalid_argument("map dimensions and resolution must be positive");
    }
    if (max_range_ <= 0.0 || max_beams_ <= 0) {
      throw std::invalid_argument("max_range and max_beams must be positive");
    }
    if (minimum_travel_distance_ < 0.0 || minimum_travel_heading_ < 0.0) {
      throw std::invalid_argument("minimum travel thresholds cannot be negative");
    }
    if (linear_noise_ < 0.0 || angular_noise_ < 0.0 || noise_floor_ < 0.0) {
      throw std::invalid_argument("motion noise values cannot be negative");
    }
    if (initial_position_variance_ <= 0.0 || initial_heading_variance_ <= 0.0 ||
      measurement_position_variance_ <= 0.0 || measurement_heading_variance_ <= 0.0)
    {
      throw std::invalid_argument("EKF variances must be positive");
    }
    if (scan_linear_window_ < 0.0 || scan_angular_window_ < 0.0 ||
      scan_linear_step_ <= 0.0 || scan_angular_step_ <= 0.0)
    {
      throw std::invalid_argument("scan matching windows and steps are invalid");
    }
    if (minimum_scan_score_ < 0.0 || minimum_scan_score_ > 1.0) {
      throw std::invalid_argument("minimum_scan_score must be in [0, 1]");
    }
    if (map_publish_interval_ < 0.0) {
      throw std::invalid_argument("map_publish_interval cannot be negative");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odom_.x = message->pose.pose.position.x;
    latest_odom_.y = message->pose.pose.position.y;
    latest_odom_.yaw = yaw_from_quaternion(message->pose.pose.orientation);
    have_odom_ = true;
  }

  std::vector<Beam> make_beams(const sensor_msgs::msg::LaserScan & scan) const
  {
    std::vector<Beam> beams;
    if (scan.ranges.empty()) {
      return beams;
    }

    const std::size_t desired = static_cast<std::size_t>(max_beams_);
    const std::size_t stride = std::max<std::size_t>(
      1, (scan.ranges.size() + desired - 1) / desired);
    const double usable_max = std::min(max_range_, static_cast<double>(scan.range_max));
    const double c_laser = std::cos(laser_yaw_);
    const double s_laser = std::sin(laser_yaw_);

    beams.reserve(std::min(desired, scan.ranges.size()));
    for (std::size_t index = 0; index < scan.ranges.size(); index += stride) {
      const double raw_range = static_cast<double>(scan.ranges[index]);
      if (std::isnan(raw_range) || raw_range < static_cast<double>(scan.range_min)) {
        continue;
      }

      const bool finite = std::isfinite(raw_range);
      const double range = finite ? std::min(raw_range, usable_max) : usable_max;
      if (range <= 0.0) {
        continue;
      }

      const double angle =
        static_cast<double>(scan.angle_min) +
        static_cast<double>(index) * static_cast<double>(scan.angle_increment);
      const double scan_x = range * std::cos(angle);
      const double scan_y = range * std::sin(angle);
      Beam beam;
      beam.x = laser_x_ + c_laser * scan_x - s_laser * scan_y;
      beam.y = laser_y_ + s_laser * scan_x + c_laser * scan_y;
      beam.hit = finite && raw_range <= usable_max && raw_range < scan.range_max;
      beams.push_back(beam);
    }
    return beams;
  }

  Pose2D odometry_delta(const Pose2D & previous, const Pose2D & current) const
  {
    const double world_dx = current.x - previous.x;
    const double world_dy = current.y - previous.y;
    const double c = std::cos(previous.yaw);
    const double s = std::sin(previous.yaw);
    Pose2D delta;
    delta.x = c * world_dx + s * world_dy;
    delta.y = -s * world_dx + c * world_dy;
    delta.yaw = normalize_angle(current.yaw - previous.yaw);
    return delta;
  }

  void predict(const Pose2D & delta)
  {
    const double yaw = state_.yaw;
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    state_.x += c * delta.x - s * delta.y;
    state_.y += s * delta.x + c * delta.y;
    state_.yaw = normalize_angle(state_.yaw + delta.yaw);

    Matrix3 jacobian = identity_matrix();
    jacobian[2] = -s * delta.x - c * delta.y;
    jacobian[5] = c * delta.x - s * delta.y;

    const double distance = std::hypot(delta.x, delta.y);
    const double position_sigma = noise_floor_ + linear_noise_ * distance;
    const double heading_sigma =
      noise_floor_ + angular_noise_ * (std::abs(delta.yaw) + 0.25 * distance);
    const double position_variance = position_sigma * position_sigma;
    const double heading_variance = heading_sigma * heading_sigma;
    const Matrix3 rotation{
      c, -s, 0.0,
      s, c, 0.0,
      0.0, 0.0, 1.0};
    const Matrix3 local_noise{
      position_variance, 0.0, 0.0,
      0.0, position_variance, 0.0,
      0.0, 0.0, heading_variance};
    const Matrix3 process_noise =
      multiply(multiply(rotation, local_noise), transpose(rotation));
    covariance_ = add(
      multiply(multiply(jacobian, covariance_), transpose(jacobian)), process_noise);
  }

  bool correct(const Pose2D & measurement)
  {
    const Matrix3 measurement_noise{
      measurement_position_variance_, 0.0, 0.0,
      0.0, measurement_position_variance_, 0.0,
      0.0, 0.0, measurement_heading_variance_};
    Matrix3 innovation_inverse{};
    if (!inverse(add(covariance_, measurement_noise), innovation_inverse)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "EKF correction skipped: singular covariance");
      return false;
    }

    const Matrix3 gain = multiply(covariance_, innovation_inverse);
    const Vector3 innovation{
      measurement.x - state_.x,
      measurement.y - state_.y,
      normalize_angle(measurement.yaw - state_.yaw)};
    const Vector3 adjustment = multiply(gain, innovation);
    state_.x += adjustment[0];
    state_.y += adjustment[1];
    state_.yaw = normalize_angle(state_.yaw + adjustment[2]);

    // Joseph form retains symmetry and positive semi-definiteness better than
    // the shorter (I-K)P expression after many scan corrections.
    const Matrix3 residual_gain = subtract(identity_matrix(), gain);
    covariance_ = add(
      multiply(multiply(residual_gain, covariance_), transpose(residual_gain)),
      multiply(multiply(gain, measurement_noise), transpose(gain)));
    for (std::size_t row = 0; row < 3; ++row) {
      covariance_[row * 3 + row] = std::max(covariance_[row * 3 + row], 1e-12);
      for (std::size_t column = row + 1; column < 3; ++column) {
        const double symmetric =
          0.5 * (covariance_[row * 3 + column] + covariance_[column * 3 + row]);
        covariance_[row * 3 + column] = symmetric;
        covariance_[column * 3 + row] = symmetric;
      }
    }
    return true;
  }

  void add_log_odds(int x, int y, double increment)
  {
    if (x < 0 || x >= map_width_ || y < 0 || y >= map_height_) {
      return;
    }
    float & value = map_[scan_matcher_.cell_index(x, y)];
    value = static_cast<float>(std::clamp(
      static_cast<double>(value) + increment, -4.0, 4.0));
  }

  void trace_beam(int start_x, int start_y, int end_x, int end_y, bool hit)
  {
    int x = start_x;
    int y = start_y;
    const int dx = std::abs(end_x - start_x);
    const int sx = start_x < end_x ? 1 : -1;
    const int dy = -std::abs(end_y - start_y);
    const int sy = start_y < end_y ? 1 : -1;
    int error = dx + dy;

    while (x != end_x || y != end_y) {
      add_log_odds(x, y, miss_log_odds_);
      const int twice_error = 2 * error;
      if (twice_error >= dy) {
        error += dy;
        x += sx;
      }
      if (twice_error <= dx) {
        error += dx;
        y += sy;
      }
    }
    add_log_odds(end_x, end_y, hit ? hit_log_odds_ : miss_log_odds_);
  }

  void update_map(const std::vector<Beam> & beams)
  {
    const double c = std::cos(state_.yaw);
    const double s = std::sin(state_.yaw);
    const double sensor_x = state_.x + c * laser_x_ - s * laser_y_;
    const double sensor_y = state_.y + s * laser_x_ + c * laser_y_;
    int start_x = 0;
    int start_y = 0;
    if (!scan_matcher_.world_to_cell(sensor_x, sensor_y, start_x, start_y)) {
      return;
    }

    for (const auto & beam : beams) {
      const double endpoint_x = state_.x + c * beam.x - s * beam.y;
      const double endpoint_y = state_.y + s * beam.x + c * beam.y;
      int end_x = 0;
      int end_y = 0;
      if (!scan_matcher_.world_to_cell(endpoint_x, endpoint_y, end_x, end_y)) {
        continue;
      }
      trace_beam(start_x, start_y, end_x, end_y, beam.hit);
    }
  }

  void publish_pose(const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = state_.x;
    message.pose.pose.position.y = state_.y;
    message.pose.pose.orientation = quaternion_from_yaw(state_.yaw);
    message.pose.covariance[0] = covariance_[0];
    message.pose.covariance[1] = covariance_[1];
    message.pose.covariance[5] = covariance_[2];
    message.pose.covariance[6] = covariance_[3];
    message.pose.covariance[7] = covariance_[4];
    message.pose.covariance[11] = covariance_[5];
    // The filter is explicitly planar, so z, roll, and pitch are constrained
    // rather than unknown. Large sentinel variances here make RViz draw an
    // enormous 3D covariance volume that obscures the map.
    message.pose.covariance[14] = 1e-9;
    message.pose.covariance[21] = 1e-9;
    message.pose.covariance[28] = 1e-9;
    message.pose.covariance[30] = covariance_[6];
    message.pose.covariance[31] = covariance_[7];
    message.pose.covariance[35] = covariance_[8];
    pose_publisher_->publish(message);
  }

  void publish_map(const builtin_interfaces::msg::Time & stamp) const
  {
    nav_msgs::msg::OccupancyGrid message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.info.resolution = static_cast<float>(resolution_);
    message.info.width = static_cast<std::uint32_t>(map_width_);
    message.info.height = static_cast<std::uint32_t>(map_height_);
    message.info.origin.position.x = origin_x_;
    message.info.origin.position.y = origin_y_;
    message.info.origin.orientation.w = 1.0;
    message.data.resize(map_.size());
    for (std::size_t index = 0; index < map_.size(); ++index) {
      if (std::abs(map_[index]) < 1e-6F) {
        message.data[index] = -1;
      } else {
        const double probability =
          1.0 / (1.0 + std::exp(-static_cast<double>(map_[index])));
        message.data[index] =
          static_cast<std::int8_t>(std::lround(100.0 * probability));
      }
    }
    map_publisher_->publish(message);
  }

  void publish_transform(const builtin_interfaces::msg::Time & stamp)
  {
    if (!tf_broadcaster_) {
      return;
    }
    const double yaw = normalize_angle(state_.yaw - latest_odom_.yaw);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x =
      state_.x - (c * latest_odom_.x - s * latest_odom_.y);
    transform.transform.translation.y =
      state_.y - (s * latest_odom_.x + c * latest_odom_.y);
    transform.transform.rotation = quaternion_from_yaw(yaw);
    tf_broadcaster_->sendTransform(transform);
  }

  bool map_publish_due(const builtin_interfaces::msg::Time & stamp)
  {
    const rclcpp::Time current(stamp);
    if (!have_map_publish_time_ ||
      (current - last_map_publish_time_).seconds() >= map_publish_interval_)
    {
      last_map_publish_time_ = current;
      have_map_publish_time_ = true;
      return true;
    }
    return false;
  }

  void publish_estimate(const builtin_interfaces::msg::Time & stamp, bool force_map = false)
  {
    publish_pose(stamp);
    publish_transform(stamp);
    if (force_map || map_publish_due(stamp)) {
      publish_map(stamp);
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    if (!have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Waiting for odometry before processing scans");
      return;
    }
    const std::vector<Beam> beams = make_beams(*scan);
    if (beams.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Received a lidar scan with no usable beams");
      return;
    }

    if (!initialized_) {
      previous_odom_ = latest_odom_;
      update_map(beams);
      initialized_ = true;
      publish_estimate(scan->header.stamp, true);
      RCLCPP_INFO(get_logger(), "EKF map initialized from the first lidar scan");
      return;
    }

    const Pose2D delta = odometry_delta(previous_odom_, latest_odom_);
    if (std::hypot(delta.x, delta.y) < minimum_travel_distance_ &&
      std::abs(delta.yaw) < minimum_travel_heading_)
    {
      publish_estimate(scan->header.stamp);
      return;
    }
    previous_odom_ = latest_odom_;

    predict(delta);
    std::vector<Beam> hit_beams;
    hit_beams.reserve(beams.size());
    for (const auto & beam : beams) {
      if (beam.hit) {
        hit_beams.push_back(beam);
      }
    }
    double matched_score = 0.0;
    const Pose2D matched_pose = scan_matcher_.match(map_, state_, hit_beams, matched_score);
    if (matched_score >= minimum_scan_score_) {
      correct(matched_pose);
    } else {
      RCLCPP_DEBUG(
        get_logger(), "Rejected lidar correction with score %.3f", matched_score);
    }
    update_map(beams);
    publish_estimate(scan->header.stamp);
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string pose_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  bool publish_tf_{false};

  double resolution_{0.05};
  int map_width_{400};
  int map_height_{400};
  double origin_x_{-10.0};
  double origin_y_{-10.0};
  double max_range_{7.5};
  int max_beams_{90};
  double minimum_travel_distance_{0.10};
  double minimum_travel_heading_{0.08};
  double linear_noise_{0.05};
  double angular_noise_{0.05};
  double noise_floor_{0.005};
  double initial_position_variance_{0.0001};
  double initial_heading_variance_{0.0001};
  double measurement_position_variance_{0.01};
  double measurement_heading_variance_{0.0064};
  double scan_linear_window_{0.20};
  double scan_angular_window_{0.16};
  double scan_linear_step_{0.05};
  double scan_angular_step_{0.04};
  double minimum_scan_score_{0.52};
  double hit_log_odds_{0.85};
  double miss_log_odds_{-0.40};
  double map_publish_interval_{1.0};
  double laser_x_{-0.032};
  double laser_y_{0.0};
  double laser_yaw_{0.0};

  Pose2D state_;
  Matrix3 covariance_{};
  std::vector<float> map_;
  ScanMatcher scan_matcher_;
  Pose2D latest_odom_;
  Pose2D previous_odom_;
  bool have_odom_{false};
  bool initialized_{false};
  bool have_map_publish_time_{false};
  rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EkfSlamNode>());
  rclcpp::shutdown();
  return 0;
}
