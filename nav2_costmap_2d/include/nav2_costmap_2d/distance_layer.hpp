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

#ifndef NAV2_COSTMAP_2D__DISTANCE_LAYER_HPP_
#define NAV2_COSTMAP_2D__DISTANCE_LAYER_HPP_

#include <limits>
#include <memory>
#include <mutex>
#include <string>
#ifdef _OPENMP
#include <omp.h>
#endif
#include <Eigen/Core>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/distance_transform.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

namespace nav2_costmap_2d
{

/**
 * @class DistanceLayer
 * @brief Layer that computes a distance-to-obstacle cost gradient using
 * Felzenszwalb-Huttenlocher distance transform.
 */
class DistanceLayer : public Layer
{
public:
  DistanceLayer();
  ~DistanceLayer();

  void onInitialize() override;
  void deactivate() override;
  void activate() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void onFootprintChanged() override;

  void matchSize() override;

  bool isClearable() override {return false;}

  void reset() override
  {
    matchSize();
    setCurrent(false);
    need_recompute_ = true;
  }

protected:
  unsigned int cellDistance(double world_dist)
  {
    return layered_costmap_->getCostmap()->cellDistance(world_dist);
  }

  int getOptimalThreadCount();

  rcl_interfaces::msg::SetParametersResult validateParameterUpdatesCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void updateParametersCallback(const std::vector<rclcpp::Parameter> & parameters);

  double max_distance_;
  double cost_scaling_distance_;
  unsigned int cell_max_distance_;
  int num_threads_;
  double resolution_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool consider_unknown_as_obstacle_;
  bool need_recompute_;
  
  Costmap2D::mutex_t * access_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_params_handler_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_params_handler_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__DISTANCE_LAYER_HPP_
