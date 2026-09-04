// Shared scan-matching primitives for the educational SLAM backends.
#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace robot_slam
{

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Beam
{
  double x{0.0};
  double y{0.0};
  bool hit{false};
};

struct ScanMatchConfig
{
  int map_width{0};
  int map_height{0};
  double origin_x{0.0};
  double origin_y{0.0};
  double resolution{0.0};
  double linear_window{0.0};
  double angular_window{0.0};
  double linear_step{0.0};
  double angular_step{0.0};
};

inline double normalize_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

class ScanMatcher
{
public:
  ScanMatcher() = default;

  explicit ScanMatcher(const ScanMatchConfig & config)
  {
    configure(config);
  }

  void configure(const ScanMatchConfig & config)
  {
    config_ = config;
    angle_offsets_.clear();
    search_candidates_.clear();
    initialize_search();
  }

  bool world_to_cell(double x, double y, int & cell_x, int & cell_y) const
  {
    cell_x = static_cast<int>(std::floor((x - config_.origin_x) / config_.resolution));
    cell_y = static_cast<int>(std::floor((y - config_.origin_y) / config_.resolution));
    return cell_x >= 0 && cell_x < config_.map_width &&
           cell_y >= 0 && cell_y < config_.map_height;
  }

  std::size_t cell_index(int x, int y) const
  {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(config_.map_width) +
           static_cast<std::size_t>(x);
  }

  Pose2D match(
    const std::vector<float> & map, const Pose2D & prediction,
    const std::vector<Beam> & hit_beams, double & matched_score) const
  {
    Pose2D best_pose = prediction;
    const ProjectedScan prediction_scan = project_scan(prediction.yaw, hit_beams);
    matched_score = scan_score(
      map, prediction.x, prediction.y, prediction_scan.beams);
    double best_objective = matched_score;
    const std::vector<ProjectedScan> projected_scans =
      project_scans(prediction, hit_beams);

    for (const auto & search : search_candidates_) {
      const auto & projected = projected_scans[search.angle_index];
      const double candidate_x = prediction.x + search.x_offset;
      const double candidate_y = prediction.y + search.y_offset;
      const double score = scan_score(map, candidate_x, candidate_y, projected.beams);
      const double objective = score - search.motion_penalty;
      if (objective > best_objective) {
        best_objective = objective;
        matched_score = score;
        best_pose = Pose2D{candidate_x, candidate_y, projected.yaw};
      }
    }
    return best_pose;
  }

private:
  struct ProjectedBeam
  {
    double cosine_x{0.0};
    double sine_y{0.0};
    double sine_x{0.0};
    double cosine_y{0.0};
  };

  struct ProjectedScan
  {
    double yaw{0.0};
    std::vector<ProjectedBeam> beams;
  };

  struct SearchCandidate
  {
    double x_offset{0.0};
    double y_offset{0.0};
    double motion_penalty{0.0};
    std::size_t angle_index{0};
  };

  void initialize_search()
  {
    // Nudge before flooring: an exact ratio such as 0.15 / 0.05 evaluates to
    // 2.9999999999999996 in binary floating point, which would silently drop
    // the outermost step and shrink the window by one increment. The epsilon
    // is small enough that a genuinely fractional ratio still floors down, so
    // the search never exceeds the configured window.
    const int linear_steps = static_cast<int>(
      std::floor(config_.linear_window / config_.linear_step + 1e-9));
    const int angular_steps = static_cast<int>(
      std::floor(config_.angular_window / config_.angular_step + 1e-9));

    angle_offsets_.reserve(static_cast<std::size_t>(2 * angular_steps + 1));
    for (int it = -angular_steps; it <= angular_steps; ++it) {
      angle_offsets_.push_back(static_cast<double>(it) * config_.angular_step);
    }

    const std::size_t candidate_count =
      static_cast<std::size_t>(2 * linear_steps + 1) *
      static_cast<std::size_t>(2 * linear_steps + 1) *
      static_cast<std::size_t>(2 * angular_steps + 1) - 1;
    search_candidates_.reserve(candidate_count);
    for (int ix = -linear_steps; ix <= linear_steps; ++ix) {
      for (int iy = -linear_steps; iy <= linear_steps; ++iy) {
        for (int it = -angular_steps; it <= angular_steps; ++it) {
          if (ix == 0 && iy == 0 && it == 0) {
            continue;
          }
          const double x_offset = static_cast<double>(ix) * config_.linear_step;
          const double y_offset = static_cast<double>(iy) * config_.linear_step;
          const double yaw_offset = static_cast<double>(it) * config_.angular_step;
          const double dx_ratio =
            config_.linear_window > 0.0 ? x_offset / config_.linear_window : 0.0;
          const double dy_ratio =
            config_.linear_window > 0.0 ? y_offset / config_.linear_window : 0.0;
          const double dt_ratio =
            config_.angular_window > 0.0 ? yaw_offset / config_.angular_window : 0.0;
          search_candidates_.push_back(SearchCandidate{
              x_offset,
              y_offset,
              0.015 * (dx_ratio * dx_ratio + dy_ratio * dy_ratio + dt_ratio * dt_ratio),
              static_cast<std::size_t>(it + angular_steps)});
        }
      }
    }
  }

  double neighborhood_probability(const std::vector<float> & map, int x, int y) const
  {
    // Sigmoid is monotonic, so selecting the largest log odds first gives the
    // same likelihood while reducing up to 25 exponential evaluations to one.
    const int min_x = std::max(0, x - 2);
    const int max_x = std::min(config_.map_width - 1, x + 2);
    const int min_y = std::max(0, y - 2);
    const int max_y = std::min(config_.map_height - 1, y + 2);
    float best_log_odds = -std::numeric_limits<float>::infinity();
    for (int cell_y = min_y; cell_y <= max_y; ++cell_y) {
      const std::size_t row =
        static_cast<std::size_t>(cell_y) * static_cast<std::size_t>(config_.map_width);
      for (int cell_x = min_x; cell_x <= max_x; ++cell_x) {
        best_log_odds = std::max(
          best_log_odds, map[row + static_cast<std::size_t>(cell_x)]);
      }
    }
    const double probability =
      1.0 / (1.0 + std::exp(-static_cast<double>(best_log_odds)));
    const bool includes_out_of_bounds =
      min_x != x - 2 || max_x != x + 2 || min_y != y - 2 || max_y != y + 2;
    return includes_out_of_bounds ? std::max(0.05, probability) : probability;
  }

  ProjectedScan project_scan(double yaw, const std::vector<Beam> & hit_beams) const
  {
    ProjectedScan projected;
    projected.yaw = yaw;
    projected.beams.reserve(hit_beams.size());
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    for (const auto & beam : hit_beams) {
      // Keep the four products separate so scoring retains the original
      // floating-point evaluation order around occupancy-cell boundaries.
      projected.beams.push_back(ProjectedBeam{
          c * beam.x, s * beam.y, s * beam.x, c * beam.y});
    }
    return projected;
  }

  std::vector<ProjectedScan> project_scans(
    const Pose2D & prediction, const std::vector<Beam> & hit_beams) const
  {
    std::vector<ProjectedScan> projected_scans;
    projected_scans.reserve(angle_offsets_.size());
    for (const double yaw_offset : angle_offsets_) {
      projected_scans.push_back(project_scan(
        normalize_angle(prediction.yaw + yaw_offset), hit_beams));
    }
    return projected_scans;
  }

  double scan_score(
    const std::vector<float> & map, double x, double y,
    const std::vector<ProjectedBeam> & beams) const
  {
    double total = 0.0;
    std::size_t hits = 0;
    for (const auto & beam : beams) {
      int cell_x = 0;
      int cell_y = 0;
      const double endpoint_x = x + beam.cosine_x - beam.sine_y;
      const double endpoint_y = y + beam.sine_x + beam.cosine_y;
      if (!world_to_cell(endpoint_x, endpoint_y, cell_x, cell_y)) {
        continue;
      }
      total += neighborhood_probability(map, cell_x, cell_y);
      ++hits;
    }
    return hits == 0 ? 0.5 : total / static_cast<double>(hits);
  }

  ScanMatchConfig config_;
  std::vector<double> angle_offsets_;
  std::vector<SearchCandidate> search_candidates_;
};

}  // namespace robot_slam
