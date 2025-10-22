// Copyright (c) 2022 Samsung Research America
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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <vector>
#include <utility>

#include "nav2_navfn_planner/parameter_handler.hpp"

namespace nav2_navfn_planner
{

using nav2::declare_parameter_if_not_declared;
using rcl_interfaces::msg::ParameterType;

ParameterHandler::ParameterHandler(
  nav2::LifecycleNode::SharedPtr node,
  std::string & plugin_name, rclcpp::Logger & logger)
{
  node_ = node;
  plugin_name_ = plugin_name;
  logger_ = logger;

  params_.tolerance = node->declare_or_get_parameter(plugin_name_ + ".tolerance", 0.5);
  params_.use_astar = node->declare_or_get_parameter(plugin_name_ + ".use_astar", false);
  params_.allow_unknown = node->declare_or_get_parameter(plugin_name_ + ".allow_unknown", true);
  params_.use_final_approach_orientation = node->declare_or_get_parameter(plugin_name_ +
    ".use_final_approach_orientation", false);
}

void ParameterHandler::activate()
{
  auto node = node_.lock();
  post_set_params_handler_ = node->add_post_set_parameters_callback(
    std::bind(
      &ParameterHandler::updateParametersCallback,
      this, std::placeholders::_1));
  on_set_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(
      &ParameterHandler::validateParameterUpdatesCallback,
      this, std::placeholders::_1));
}

void ParameterHandler::deactivate()
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

ParameterHandler::~ParameterHandler()
{
}

rcl_interfaces::msg::SetParametersResult ParameterHandler::validateParameterUpdatesCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if (param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".tolerance" &&
        parameter.as_double() < 0.0)
      {
        RCLCPP_WARN(
        logger_, "The value of tolerance is incorrectly set, "
        "it should be >=0. Ignoring parameter update.");
        result.successful = false;
      }
    }
  }
  return result;
}

void
ParameterHandler::updateParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & param_type = parameter.get_type();
    const auto & param_name = parameter.get_name();
    if(param_name.find(plugin_name_ + ".") != 0) {
      continue;
    }
    if (param_type == ParameterType::PARAMETER_DOUBLE) {
      if (param_name == plugin_name_ + ".tolerance") {
        params_.tolerance = parameter.as_double();
      }
    } else if (param_type == ParameterType::PARAMETER_BOOL) {
      if (param_name == plugin_name_ + ".use_astar") {
        params_.use_astar = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".allow_unknown") {
        params_.allow_unknown = parameter.as_bool();
      } else if (param_name == plugin_name_ + ".use_final_approach_orientation") {
        params_.use_final_approach_orientation = parameter.as_bool();
      }
    }
  }
}

}  // namespace nav2_navfn_planner
