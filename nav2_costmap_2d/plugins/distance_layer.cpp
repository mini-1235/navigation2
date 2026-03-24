// Copyright (c) 2026 Maurice Alexander Purnawan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "nav2_costmap_2d/distance_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#ifdef _OPENMP
#include <omp.h>
#endif

#include "nav2_costmap_2d/distance_transform.hpp"
#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::DistanceLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

DistanceLayer::DistanceLayer()
: cell_max_distance_(0),
  resolution_(0.0),
  last_min_x_(std::numeric_limits<double>::lowest()),
  last_min_y_(std::numeric_limits<double>::lowest()),
  last_max_x_(std::numeric_limits<double>::max()),
  last_max_y_(std::numeric_limits<double>::max()),
  need_recompute_(false)
{
  access_ = new Costmap2D::mutex_t();
}

DistanceLayer::~DistanceLayer()
{
  delete access_;
}

void DistanceLayer::onInitialize()
{
  {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error{"Failed to lock node"};
    }
    enabled_ = node->declare_or_get_parameter(name_ + "." + "enabled", true);
    max_distance_ = node->declare_or_get_parameter(name_ + "." + "max_distance", 3.0);
    cost_scaling_distance_ = node->declare_or_get_parameter(name_ + "." + "cost_scaling_distance", 0.5);
    num_threads_ = node->declare_or_get_parameter(name_ + "." + "num_threads", -1);
    consider_unknown_as_obstacle_ = node->declare_or_get_parameter(name_ + "." + "consider_unknown_as_obstacle", true);
  }

  setCurrent(true);
  need_recompute_ = false;
  matchSize();
}

void DistanceLayer::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(&DistanceLayer::updateParametersCallback, this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&DistanceLayer::validateParameterUpdatesCallback, this, std::placeholders::_1));
}

void DistanceLayer::deactivate()
{
  auto node = node_.lock();
  if (post_set_params_handler_ && node) {
    node->remove_post_set_parameters_callback(post_set_params_handler_.get());
  }
  post_set_params_handler_.reset();
  if (on_set_params_handler_ && node) {
    node->remove_on_set_parameters_callback(on_set_params_handler_.get());
  }
  on_set_params_handler_.reset();
}

void DistanceLayer::matchSize()
{
  std::lock_guard<Costmap2D::mutex_t> guard(*access_);
  nav2_costmap_2d::Costmap2D * costmap = layered_costmap_->getCostmap();
  resolution_ = costmap->getResolution();
  cell_max_distance_ = cellDistance(max_distance_);
}

void DistanceLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*access_);
  if (need_recompute_) {
    last_min_x_ = last_min_y_ = std::numeric_limits<double>::max();
    last_max_x_ = last_max_y_ = std::numeric_limits<double>::lowest();

    *min_x = std::numeric_limits<double>::lowest();
    *min_y = std::numeric_limits<double>::lowest();
    *max_x = std::numeric_limits<double>::max();
    *max_y = std::numeric_limits<double>::max();
    need_recompute_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x) - max_distance_;
    *min_y = std::min(tmp_min_y, *min_y) - max_distance_;
    *max_x = std::max(tmp_max_x, *max_x) + max_distance_;
    *max_y = std::max(tmp_max_y, *max_y) + max_distance_;
  }
}

void DistanceLayer::onFootprintChanged()
{
  need_recompute_ = true;
}

int DistanceLayer::getOptimalThreadCount()
{
#ifdef _OPENMP
  if (num_threads_ > 0) {
    RCLCPP_INFO_ONCE(logger_, "OpenMP: Using configured num_threads: %d", num_threads_);
    return num_threads_;
  }

  int cpu_cores = omp_get_max_threads();
  int optimal = std::max(1, cpu_cores / 2);

  RCLCPP_INFO_ONCE(
    logger_,
    "OpenMP: %d cores available, using %d threads (auto)",
    cpu_cores, optimal);

  return optimal;
#else
  return 1;
#endif
}

void DistanceLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*access_);
  if (!enabled_) {
    return;
  }

  unsigned char * master_array = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  min_i = std::max(0, min_i);
  min_j = std::max(0, min_j);
  max_i = std::min(static_cast<int>(size_x), max_i);
  max_j = std::min(static_cast<int>(size_y), max_j);

  const int padding = static_cast<int>(cell_max_distance_);
  int roi_min_i = std::max(0, min_i - padding);
  int roi_min_j = std::max(0, min_j - padding);
  int roi_max_i = std::min(static_cast<int>(size_x), max_i + padding);
  int roi_max_j = std::min(static_cast<int>(size_y), max_j + padding);

  const int roi_width = roi_max_i - roi_min_i;
  const int roi_height = roi_max_j - roi_min_j;

  MatrixXfRM distance_map(roi_height, roi_width);

#ifdef _OPENMP
  const int num_threads = getOptimalThreadCount();
  #pragma omp parallel for num_threads(num_threads) schedule(dynamic, 16)
#endif
  for (int y = 0; y < roi_height; y++) {
    const int src_y = y + roi_min_j;
    for (int x = 0; x < roi_width; x++) {
      const int src_x = x + roi_min_i;
      const unsigned char cell = master_array[src_y * size_x + src_x];
      if (cell == LETHAL_OBSTACLE) {
        distance_map(y, x) = 0.0f;
      } else if (cell == NO_INFORMATION && consider_unknown_as_obstacle_) {
        distance_map(y, x) = 0.0f;
      } else {
        distance_map(y, x) = DistanceTransform::DT_INF;
      }
    }
  }

  DistanceTransform::distanceTransform2D(distance_map, roi_height, roi_width);

  for (int y = 0; y < roi_height; y++) {
    const int src_y = y + roi_min_j;
    for (int x = 0; x < roi_width; x++) {
      const int src_x = x + roi_min_i;
      float dist = distance_map(y, x) * resolution_;
      // If distance is greater than max_distance_, treat it as free space
      if (dist > max_distance_) {
        master_array[src_y * size_x + src_x] = FREE_SPACE;
      } else if (dist < cost_scaling_distance_) {
        // If distance is less than cost scaling distance, we assign it as inscribed inflated obstacle
        master_array[src_y * size_x + src_x] = INSCRIBED_INFLATED_OBSTACLE;
      } else {
        // Scale cost linearly between INSCRIBED_INFLATED_OBSTACLE and FREE
        float scale = (max_distance_ - dist) / (max_distance_ - cost_scaling_distance_);
        unsigned char cost = static_cast<unsigned char>(
          MAX_NON_OBSTACLE * scale);
        master_array[src_y * size_x + src_x] = cost;
      }
    }
  }

  setCurrent(true);
}

rcl_interfaces::msg::SetParametersResult DistanceLayer::validateParameterUpdatesCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (parameter.as_double() < 0.0) {
        RCLCPP_WARN(
          logger_,
          "The value of parameter '%s' is incorrectly set to %f, it should be >=0. "
          "Ignoring parameter update.",
          param_name.c_str(), parameter.as_double());
        result.successful = false;
      }
    }
  }
  return result;
}

void DistanceLayer::updateParametersCallback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*access_);

  bool need_resize = false;

  for (const auto & parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(name_ + ".") != 0) {
      continue;
    }

    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == name_ + "." + "max_distance" &&
        max_distance_ != parameter.as_double())
      {
        max_distance_ = parameter.as_double();
        need_recompute_ = true;
        need_resize = true;
        setCurrent(false);
      } else if (param_name == name_ + "." + "cost_scaling_distance" && // NOLINT
        cost_scaling_distance_ != parameter.as_double())
      {
        cost_scaling_distance_ = parameter.as_double();
        need_recompute_ = true;
        setCurrent(false);
      }
    } else if (param_type == ParameterType::PARAMETER_INTEGER) {
      if (param_name == name_ + "." + "num_threads" && num_threads_ != parameter.as_int()) {
        int new_value = parameter.as_int();
#ifdef _OPENMP
        if (new_value < -1) {
          RCLCPP_WARN(
            logger_,
            "Invalid num_threads value %d, must be -1 (auto) or > 0. Ignoring.",
            new_value);
        } else {
          int available_cores = omp_get_max_threads();
          if (new_value > available_cores) {
            RCLCPP_WARN(
              logger_,
              "num_threads=%d exceeds available cores (%d). Ignoring.",
              new_value, available_cores);
          } else {
            num_threads_ = new_value;
            RCLCPP_INFO(
              logger_,
              "Updated num_threads to %d %s",
              num_threads_,
              num_threads_ == -1 ? "(auto)" : "");
          }
        }
#else
        RCLCPP_WARN(
          logger_,
          "num_threads parameter ignored - OpenMP support not available. "
          "Distance layer will use single thread.");
        num_threads_ = new_value;
#endif
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == name_ + "." + "enabled" && enabled_ != parameter.as_bool()) {
        enabled_ = parameter.as_bool();
        need_recompute_ = true;
        setCurrent(false);
      } else if (param_name == name_ + "." + "consider_unknown_as_obstacle" &&
        consider_unknown_as_obstacle_ != parameter.as_bool())
      {
        consider_unknown_as_obstacle_ = parameter.as_bool();
        need_recompute_ = true;
        setCurrent(false);
      }
    }
  }

  if (need_resize) {
    matchSize();
  }
}

}  // namespace nav2_costmap_2d
