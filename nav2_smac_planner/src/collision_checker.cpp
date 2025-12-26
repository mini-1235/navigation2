// Copyright (c) 2021, Samsung Research America
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
// limitations under the License. Reserved.

#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_util/line_iterator.hpp"

namespace nav2_smac_planner
{

GridCollisionChecker::GridCollisionChecker(
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
  unsigned int num_quantizations,
  nav2::LifecycleNode::SharedPtr node)
: FootprintCollisionChecker(costmap_ros ? costmap_ros->getCostmap() : nullptr)
{
  if (node) {
    clock_ = node->get_clock();
    logger_ = node->get_logger();
  }

  if (costmap_ros) {
    costmap_ros_ = costmap_ros;
  }

  // Convert number of regular bins into angles
  float bin_size = 2 * M_PI / static_cast<float>(num_quantizations);
  angles_.reserve(num_quantizations);
  for (unsigned int i = 0; i != num_quantizations; i++) {
    angles_.push_back(bin_size * i);
  }
}

// GridCollisionChecker::GridCollisionChecker(
//   nav2_costmap_2d::Costmap2D * costmap,
//   std::vector<float> & angles)
// : FootprintCollisionChecker(costmap),
//   angles_(angles)
// {
// }

void GridCollisionChecker::setFootprint(
  const nav2_costmap_2d::Footprint & footprint,
  const bool & radius,
  const double & possible_collision_cost)
{
  possible_collision_cost_ = static_cast<float>(possible_collision_cost);
  if (possible_collision_cost_ <= 0.0f) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 1000,
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  footprint_is_radius_ = radius;

  // Use radius, no caching required
  if (radius) {
    return;
  }

  // No change, no updates required
  if (footprint == unoriented_footprint_) {
    return;
  }

  // oriented_footprints_.clear();
  // oriented_footprints_.reserve(angles_.size());
  precomputed_map_coords_.clear();
  precomputed_map_coords_.reserve(angles_.size());
  double sin_th, cos_th;
  geometry_msgs::msg::Point new_pt;
  const unsigned int footprint_size = footprint.size();

  // Precompute the orientation bins for checking to use
  for (unsigned int i = 0; i != angles_.size(); i++) {
    sin_th = sin(angles_[i]);
    cos_th = cos(angles_[i]);
    nav2_costmap_2d::Footprint oriented_footprint;
    oriented_footprint.reserve(footprint_size);
    std::vector<std::pair<int, int>> map_points;
    map_points.reserve(footprint_size);

    for (unsigned int j = 0; j < footprint_size; j++) {
      new_pt.x = footprint[j].x * cos_th - footprint[j].y * sin_th;
      new_pt.y = footprint[j].x * sin_th + footprint[j].y * cos_th;
      // unsigned int mx, my;  
      oriented_footprint.push_back(new_pt);
      // if (worldToMap(new_pt.x, new_pt.y, mx, my)) {  
      //   map_points.emplace_back(static_cast<int>(mx), static_cast<int>(my));  
      // }
    }

    // oriented_footprints_.push_back(oriented_footprint);

    std::vector<std::pair<int, int>> line_cells;
    // For each edge in the footprint, get line iterator cells
    for (unsigned int j = 0; j < oriented_footprint.size(); j++) {  
      unsigned int j_next = (j + 1) % oriented_footprint.size();   // Wrap around for closed polygon  
      // Convert to map coordinates (assuming robot at origin)  
      int x0 = static_cast<int>(oriented_footprint[j].x / costmap_->getResolution());  
      int y0 = static_cast<int>(oriented_footprint[j].y / costmap_->getResolution());  
      int x1 = static_cast<int>(oriented_footprint[j_next].x / costmap_->getResolution());  
      int y1 = static_cast<int>(oriented_footprint[j_next].y / costmap_->getResolution());  
        
      // Use line iterator to get all cells along the edge  
      for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {  
        line_cells.emplace_back(line.getX(), line.getY());  
      }  
    }
    precomputed_map_coords_.push_back(line_cells);
  }

  unoriented_footprint_ = footprint;
}

bool GridCollisionChecker::inCollision(
  const float & x,
  const float & y,
  const float & angle_bin,
  const bool & traverse_unknown)
{
  // Check to make sure cell is inside the map
  if (outsideRange(costmap_->getSizeInCellsX(), x) ||
    outsideRange(costmap_->getSizeInCellsY(), y))
  {
    return true;
  }

  // Assumes setFootprint already set
  center_cost_ = static_cast<float>(costmap_->getCost(
      static_cast<unsigned int>(x + 0.5f), static_cast<unsigned int>(y + 0.5f)));

  if (!footprint_is_radius_) {
    // if footprint, then we check for the footprint's points, but first see
    // if the robot is even potentially in an inscribed collision
    if (center_cost_ < possible_collision_cost_ && possible_collision_cost_ > 0.0f) {
      // std::cout << "center cost" << center_cost_ << std::endl;
      // std::cout << "condition1" << std::endl;
      return false;
    }

    // If its inscribed, in collision, or unknown in the middle,
    // no need to even check the footprint, its invalid
    if (center_cost_ == UNKNOWN_COST && !traverse_unknown) {
      // std::cout << "condition2" << std::endl;
      return true;
    }

    if (center_cost_ == INSCRIBED_COST || center_cost_ == OCCUPIED_COST) {
      // std::cout << "condition3" << std::endl;
      return true;
    }

    // if possible inscribed, need to check actual footprint pose.
    // Use precomputed oriented footprints are done on initialization,
    // offset by translation value to collision check
    double wx, wy;
    costmap_->mapToWorld(static_cast<double>(x), static_cast<double>(y), wx, wy);
    geometry_msgs::msg::Point new_pt;
    // const nav2_costmap_2d::Footprint & oriented_footprint = oriented_footprints_[angle_bin];
    nav2_costmap_2d::Footprint current_footprint;
    // std::cout << "wx" << wx << "wy" << wy << std::endl;
    // std::cout << "footprint" << std::endl;
    // current_footprint.reserve(oriented_footprint.size());
    // for (unsigned int i = 0; i < oriented_footprint.size(); ++i) {
    //   new_pt.x = wx + oriented_footprint[i].x;
    //   new_pt.y = wy + oriented_footprint[i].y;
    // //   // std::cout << "wx" << new_pt.x << "wy" << new_pt.y << std::endl;
    //   current_footprint.push_back(new_pt);
    // }
    // bool res1 = false;
    // bool res2;
    // std::cout << "robotx" << static_cast<int>(x) << "roboty" << static_cast<int>(y) << std::endl;
    // std::cout << "robotx" << static_cast<int>(x+0.5) << "roboty" << static_cast<int>(y+0.5) << std::endl;
    const auto & map_coords = precomputed_map_coords_[angle_bin];
    const int robot_x = static_cast<int>(x+0.5);
    const int robot_y = static_cast<int>(y+0.5);
    const int max_x = static_cast<int>(costmap_->getSizeInCellsX());
    const int max_y = static_cast<int>(costmap_->getSizeInCellsY());
    // std::cout << "map coords size" << map_coords.size() << std::endl;
    for (const auto & coord : map_coords) {
      // std::cout << "mx" << coord.first << "my" << coord.second << std::endl;
      int check_x = robot_x + coord.first;
      int check_y = robot_y + coord.second;

      // Check bounds
      if (check_x < 0 || check_x >= max_x ||
        check_y < 0 || check_y >= max_y)
      {
        // std::cout << "out of bounds" << std::endl;
        // res1 = true;
        return true;
      }

      unsigned char cost = costmap_->getCost(check_x, check_y);
      // std::cout << "cost" << static_cast<int>(cost) << std::endl;
      if (cost == UNKNOWN_COST && !traverse_unknown) {
        // std::cout << "unknown cost" << std::endl;
        // res1 = true;
        return true;
      }

      if (cost >= OCCUPIED_COST) {
        // std::cout << "larger than inscribed" << std::endl;
        // res1 = true;
        return true;
      }
    }
    // std::cout << "returning false" << std::endl;
    return false;
    // float footprint_cost = static_cast<float>(footprintCost(current_footprint));

    // if (footprint_cost == UNKNOWN_COST && traverse_unknown) {
    //   // std::cout << "traveling unknown" << std::endl;
    //   res2 = false;
    // }

    // // if occupied or unknown and not to traverse unknown space
    // std::cout << "footprint cost" << footprint_cost << std::endl;
    // res2 = footprint_cost >= OCCUPIED_COST;
    // if (res1 != res2){
    //   std::cout << "result different " << std::endl;
    //   std::cout << "res1" << res1 << "res2" << res2 << std::endl;
    //   exit(1);
    // }
    // return res1;
  } else {
    // std::cout << "center cost" << std::endl;
    // if radius, then we can check the center of the cost assuming inflation is used
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }

    // if occupied or unknown and not to traverse unknown space
    return center_cost_ >= INSCRIBED_COST;
  }
}

bool GridCollisionChecker::inCollision(
  const unsigned int & i,
  const bool & traverse_unknown)
{
  center_cost_ = costmap_->getCost(i);
  if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
    return false;
  }

  // if occupied or unknown and not to traverse unknown space
  return center_cost_ >= INSCRIBED_COST;
}

float GridCollisionChecker::getCost()
{
  // Assumes inCollision called prior
  return static_cast<float>(center_cost_);
}

bool GridCollisionChecker::outsideRange(const unsigned int & max, const float & value)
{
  return value < 0.0f || value > max;
}

}  // namespace nav2_smac_planner
