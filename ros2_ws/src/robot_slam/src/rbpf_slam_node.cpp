#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose_array.hpp"
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

struct Particle
{
  Pose2D pose;
  double weight{1.0};
  std::vector<float> map;
};

struct PoseEstimate
{
  Pose2D pose;
  double covariance_xx{0.0};
  double covariance_xy{0.0};
  double covariance_xt{0.0};
  double covariance_yy{0.0};
  double covariance_yt{0.0};
  double covariance_tt{0.0};
};

}  // namespace

class RbpfSlamNode : public rclcpp::Node
{
public:
  RbpfSlamNode()
  : Node("rbpf_slam"),
    rng_(static_cast<std::mt19937::result_type>(declare_parameter<int>("random_seed", 42)))
  {
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
    map_topic_ = declare_parameter<std::string>("map_topic", "/rbpf/map");
    pose_topic_ = declare_parameter<std::string>("pose_topic", "/rbpf/pose");
    particle_topic_ =
      declare_parameter<std::string>("particle_topic", "/rbpf/particles");
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);

    particle_count_ = declare_parameter<int>("particle_count", 30);
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
    scan_linear_window_ = declare_parameter<double>("scan_match_linear_window", 0.15);
    scan_angular_window_ = declare_parameter<double>("scan_match_angular_window", 0.12);
    scan_linear_step_ = declare_parameter<double>("scan_match_linear_step", 0.05);
    scan_angular_step_ = declare_parameter<double>("scan_match_angular_step", 0.04);
    likelihood_gain_ = declare_parameter<double>("likelihood_gain", 30.0);
    resample_threshold_ = declare_parameter<double>("resample_threshold", 0.5);
    hit_log_odds_ = declare_parameter<double>("hit_log_odds", 0.85);
    miss_log_odds_ = declare_parameter<double>("miss_log_odds", -0.40);
    map_publish_interval_ = declare_parameter<double>("map_publish_interval", 1.0);

    // The current robot model places base_scan 3.2 cm behind base_link. Keeping
    // this configurable also lets the node work with a physical robot without
    // embedding the simulated URDF in the estimator.
    laser_x_ = declare_parameter<double>("laser_x", -0.032);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    validate_parameters();
    scan_matcher_.configure(ScanMatchConfig{
      map_width_, map_height_, origin_x_, origin_y_, resolution_,
      scan_linear_window_, scan_angular_window_, scan_linear_step_, scan_angular_step_});
    initialize_particles();

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    map_publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, map_qos);
    pose_publisher_ =
      create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
    particle_publisher_ =
      create_publisher<geometry_msgs::msg::PoseArray>(particle_topic_, 10);
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, 20,
      std::bind(&RbpfSlamNode::odom_callback, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&RbpfSlamNode::scan_callback, this, std::placeholders::_1));

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    RCLCPP_INFO(
      get_logger(),
      "RBPF SLAM ready with %d particles; publishing %s, %s, and %s in frame %s%s",
      particle_count_, map_topic_.c_str(), pose_topic_.c_str(), particle_topic_.c_str(),
      map_frame_.c_str(),
      publish_tf_ ? " with map-to-odom TF" : " without TF (comparison mode)");
  }

private:
  void validate_parameters() const
  {
    if (particle_count_ < 2) {
      throw std::invalid_argument("particle_count must be at least 2");
    }
    if (resolution_ <= 0.0 || map_width_ <= 0 || map_height_ <= 0) {
      throw std::invalid_argument("map dimensions and resolution must be positive");
    }
    if (max_range_ <= 0.0 || max_beams_ <= 0) {
      throw std::invalid_argument("max_range and max_beams must be positive");
    }
    if (scan_linear_window_ < 0.0 || scan_angular_window_ < 0.0 ||
      scan_linear_step_ <= 0.0 || scan_angular_step_ <= 0.0)
    {
      throw std::invalid_argument("scan matching windows and steps are invalid");
    }
    if (resample_threshold_ <= 0.0 || resample_threshold_ > 1.0) {
      throw std::invalid_argument("resample_threshold must be in (0, 1]");
    }
  }

  void initialize_particles()
  {
    const std::size_t cell_count =
      static_cast<std::size_t>(map_width_) * static_cast<std::size_t>(map_height_);
    particles_.resize(static_cast<std::size_t>(particle_count_));
    for (auto & particle : particles_) {
      particle.weight = 1.0 / static_cast<double>(particle_count_);
      particle.map.assign(cell_count, 0.0F);
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

  void add_log_odds(std::vector<float> & map, int x, int y, double increment) const
  {
    if (x < 0 || x >= map_width_ || y < 0 || y >= map_height_) {
      return;
    }
    float & value = map[scan_matcher_.cell_index(x, y)];
    value = static_cast<float>(std::clamp(
      static_cast<double>(value) + increment, -4.0, 4.0));
  }

  void trace_beam(
    std::vector<float> & map, int start_x, int start_y, int end_x, int end_y,
    bool hit) const
  {
    int x = start_x;
    int y = start_y;
    const int dx = std::abs(end_x - start_x);
    const int sx = start_x < end_x ? 1 : -1;
    const int dy = -std::abs(end_y - start_y);
    const int sy = start_y < end_y ? 1 : -1;
    int error = dx + dy;

    while (x != end_x || y != end_y) {
      add_log_odds(map, x, y, miss_log_odds_);
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
    if (hit) {
      add_log_odds(map, end_x, end_y, hit_log_odds_);
    } else {
      add_log_odds(map, end_x, end_y, miss_log_odds_);
    }
  }

  void update_map(Particle & particle, const std::vector<Beam> & beams) const
  {
    const double c = std::cos(particle.pose.yaw);
    const double s = std::sin(particle.pose.yaw);
    const double sensor_x = particle.pose.x + c * laser_x_ - s * laser_y_;
    const double sensor_y = particle.pose.y + s * laser_x_ + c * laser_y_;
    int start_x = 0;
    int start_y = 0;
    scan_matcher_.world_to_cell(sensor_x, sensor_y, start_x, start_y);

    for (const auto & beam : beams) {
      const double endpoint_x = particle.pose.x + c * beam.x - s * beam.y;
      const double endpoint_y = particle.pose.y + s * beam.x + c * beam.y;
      int end_x = 0;
      int end_y = 0;
      scan_matcher_.world_to_cell(endpoint_x, endpoint_y, end_x, end_y);
      trace_beam(particle.map, start_x, start_y, end_x, end_y, beam.hit);
    }
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

  Pose2D sample_motion(const Pose2D & pose, const Pose2D & delta)
  {
    const double distance = std::hypot(delta.x, delta.y);
    const double position_sigma = noise_floor_ + linear_noise_ * distance;
    const double angle_sigma =
      noise_floor_ + angular_noise_ * (std::abs(delta.yaw) + 0.25 * distance);
    std::normal_distribution<double> position_noise(0.0, position_sigma);
    std::normal_distribution<double> angle_noise(0.0, angle_sigma);

    const double noisy_x = delta.x + position_noise(rng_);
    const double noisy_y = delta.y + position_noise(rng_);
    const double noisy_yaw = delta.yaw + angle_noise(rng_);
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);

    Pose2D result;
    result.x = pose.x + c * noisy_x - s * noisy_y;
    result.y = pose.y + s * noisy_x + c * noisy_y;
    result.yaw = normalize_angle(pose.yaw + noisy_yaw);
    return result;
  }

  void normalize_weights(const std::vector<double> & scores)
  {
    std::vector<double> log_weights(particles_.size());
    double maximum = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      log_weights[i] =
        std::log(std::max(particles_[i].weight, 1e-300)) +
        likelihood_gain_ * (scores[i] - 0.5);
      maximum = std::max(maximum, log_weights[i]);
    }

    double sum = 0.0;
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      particles_[i].weight = std::exp(log_weights[i] - maximum);
      sum += particles_[i].weight;
    }
    if (!std::isfinite(sum) || sum <= 0.0) {
      const double uniform = 1.0 / static_cast<double>(particles_.size());
      for (auto & particle : particles_) {
        particle.weight = uniform;
      }
      return;
    }
    for (auto & particle : particles_) {
      particle.weight /= sum;
    }
  }

  double effective_particle_count() const
  {
    double squared_sum = 0.0;
    for (const auto & particle : particles_) {
      squared_sum += particle.weight * particle.weight;
    }
    return squared_sum > 0.0 ? 1.0 / squared_sum : 0.0;
  }

  void systematic_resample()
  {
    const std::size_t count = particles_.size();
    std::vector<double> cumulative(count);
    cumulative[0] = particles_[0].weight;
    for (std::size_t i = 1; i < count; ++i) {
      cumulative[i] = cumulative[i - 1] + particles_[i].weight;
    }
    cumulative.back() = 1.0;

    std::uniform_real_distribution<double> initial(0.0, 1.0 / static_cast<double>(count));
    const double offset = initial(rng_);
    std::vector<Particle> resampled;
    resampled.reserve(count);
    std::size_t source = 0;
    for (std::size_t i = 0; i < count; ++i) {
      const double target = offset + static_cast<double>(i) / static_cast<double>(count);
      while (source + 1 < count && target > cumulative[source]) {
        ++source;
      }
      resampled.push_back(particles_[source]);
      resampled.back().weight = 1.0 / static_cast<double>(count);
    }
    particles_ = std::move(resampled);
  }

  PoseEstimate estimate_pose() const
  {
    PoseEstimate estimate;
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    for (const auto & particle : particles_) {
      estimate.pose.x += particle.weight * particle.pose.x;
      estimate.pose.y += particle.weight * particle.pose.y;
      sin_sum += particle.weight * std::sin(particle.pose.yaw);
      cos_sum += particle.weight * std::cos(particle.pose.yaw);
    }
    estimate.pose.yaw = std::atan2(sin_sum, cos_sum);

    for (const auto & particle : particles_) {
      const double dx = particle.pose.x - estimate.pose.x;
      const double dy = particle.pose.y - estimate.pose.y;
      const double dt = normalize_angle(particle.pose.yaw - estimate.pose.yaw);
      estimate.covariance_xx += particle.weight * dx * dx;
      estimate.covariance_xy += particle.weight * dx * dy;
      estimate.covariance_xt += particle.weight * dx * dt;
      estimate.covariance_yy += particle.weight * dy * dy;
      estimate.covariance_yt += particle.weight * dy * dt;
      estimate.covariance_tt += particle.weight * dt * dt;
    }
    return estimate;
  }

  std::size_t best_particle_index() const
  {
    return static_cast<std::size_t>(std::distance(
      particles_.begin(),
      std::max_element(
        particles_.begin(), particles_.end(),
               [](const Particle & left, const Particle & right) {
                 return left.weight < right.weight;
        })));
  }

  void publish_pose(
    const PoseEstimate & estimate, const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.pose.pose.position.x = estimate.pose.x;
    message.pose.pose.position.y = estimate.pose.y;
    message.pose.pose.orientation = quaternion_from_yaw(estimate.pose.yaw);
    message.pose.covariance[0] = estimate.covariance_xx;
    message.pose.covariance[1] = estimate.covariance_xy;
    message.pose.covariance[5] = estimate.covariance_xt;
    message.pose.covariance[6] = estimate.covariance_xy;
    message.pose.covariance[7] = estimate.covariance_yy;
    message.pose.covariance[11] = estimate.covariance_yt;
    message.pose.covariance[14] = 1e6;
    message.pose.covariance[21] = 1e6;
    message.pose.covariance[28] = 1e6;
    message.pose.covariance[30] = estimate.covariance_xt;
    message.pose.covariance[31] = estimate.covariance_yt;
    message.pose.covariance[35] = estimate.covariance_tt;
    pose_publisher_->publish(message);
  }

  void publish_particles(const builtin_interfaces::msg::Time & stamp) const
  {
    geometry_msgs::msg::PoseArray message;
    message.header.stamp = stamp;
    message.header.frame_id = map_frame_;
    message.poses.reserve(particles_.size());
    for (const auto & particle : particles_) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = particle.pose.x;
      pose.position.y = particle.pose.y;
      pose.orientation = quaternion_from_yaw(particle.pose.yaw);
      message.poses.push_back(pose);
    }
    particle_publisher_->publish(message);
  }

  void publish_map(
    const std::vector<float> & map, const builtin_interfaces::msg::Time & stamp) const
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
    message.data.resize(map.size());
    for (std::size_t i = 0; i < map.size(); ++i) {
      if (std::abs(map[i]) < 1e-6F) {
        message.data[i] = -1;
      } else {
        const double probability = 1.0 / (1.0 + std::exp(-static_cast<double>(map[i])));
        message.data[i] = static_cast<std::int8_t>(std::lround(100.0 * probability));
      }
    }
    map_publisher_->publish(message);
  }

  void publish_transform(
    const PoseEstimate & estimate, const builtin_interfaces::msg::Time & stamp)
  {
    if (!tf_broadcaster_) {
      return;
    }
    const double yaw = normalize_angle(estimate.pose.yaw - latest_odom_.yaw);
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = map_frame_;
    transform.child_frame_id = odom_frame_;
    transform.transform.translation.x =
      estimate.pose.x - (c * latest_odom_.x - s * latest_odom_.y);
    transform.transform.translation.y =
      estimate.pose.y - (s * latest_odom_.x + c * latest_odom_.y);
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

  void publish_estimate(
    const builtin_interfaces::msg::Time & stamp, bool force_map = false)
  {
    const PoseEstimate estimate = estimate_pose();
    publish_pose(estimate, stamp);
    publish_particles(stamp);
    publish_transform(estimate, stamp);
    if (force_map || map_publish_due(stamp)) {
      publish_map(particles_[best_particle_index()].map, stamp);
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
      update_map(particles_.front(), beams);
      for (std::size_t i = 1; i < particles_.size(); ++i) {
        particles_[i].map = particles_.front().map;
      }
      initialized_ = true;
      publish_estimate(scan->header.stamp, true);
      RCLCPP_INFO(get_logger(), "RBPF map initialized from the first lidar scan");
      return;
    }

    const Pose2D delta = odometry_delta(previous_odom_, latest_odom_);
    if (std::hypot(delta.x, delta.y) < minimum_travel_distance_ &&
      std::abs(delta.yaw) < minimum_travel_heading_)
    {
      // TF is not latched. Continue publishing the current estimate so a
      // listener that starts while the robot is stationary can join the tree.
      publish_estimate(scan->header.stamp);
      return;
    }
    previous_odom_ = latest_odom_;

    std::vector<Beam> hit_beams;
    hit_beams.reserve(beams.size());
    for (const auto & beam : beams) {
      if (beam.hit) {
        hit_beams.push_back(beam);
      }
    }

    std::vector<double> scores(particles_.size(), 0.5);
    for (std::size_t i = 0; i < particles_.size(); ++i) {
      const Pose2D prediction = sample_motion(particles_[i].pose, delta);
      particles_[i].pose =
        scan_matcher_.match(particles_[i].map, prediction, hit_beams, scores[i]);
    }
    normalize_weights(scores);
    for (auto & particle : particles_) {
      update_map(particle, beams);
    }

    publish_estimate(scan->header.stamp);

    const double effective = effective_particle_count();
    if (effective < resample_threshold_ * static_cast<double>(particles_.size())) {
      systematic_resample();
      RCLCPP_DEBUG(get_logger(), "Resampled particles at N_eff=%.2f", effective);
    }
  }

  std::string scan_topic_;
  std::string odom_topic_;
  std::string map_topic_;
  std::string pose_topic_;
  std::string particle_topic_;
  std::string map_frame_;
  std::string odom_frame_;
  bool publish_tf_{false};

  int particle_count_{30};
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
  double scan_linear_window_{0.15};
  double scan_angular_window_{0.12};
  double scan_linear_step_{0.05};
  double scan_angular_step_{0.04};
  double likelihood_gain_{30.0};
  double resample_threshold_{0.5};
  double hit_log_odds_{0.85};
  double miss_log_odds_{-0.40};
  double map_publish_interval_{1.0};
  double laser_x_{-0.032};
  double laser_y_{0.0};
  double laser_yaw_{0.0};

  std::vector<Particle> particles_;
  ScanMatcher scan_matcher_;
  std::mt19937 rng_;
  Pose2D latest_odom_;
  Pose2D previous_odom_;
  bool have_odom_{false};
  bool initialized_{false};
  bool have_map_publish_time_{false};
  rclcpp::Time last_map_publish_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RbpfSlamNode>());
  rclcpp::shutdown();
  return 0;
}
