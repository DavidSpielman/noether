/**
 * @file plane_slicer_raster_planner.h
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <noether_tpp/core/tool_path_planner.h>
#include <string.h>
#include <geometry_msgs/msg/pose.h>

namespace noether
{
/**
 * @brief An implementation of a tool path planner that generates a toolpath on a flat plane.
 */
class FlatPlaneToolPathPlanner : public ToolPathPlanner
{
public:
  FlatPlaneToolPathPlanner(double plane_x_length, double plane_y_length, double z_offset, double spacing, geometry_msgs::msg::Pose reference_frame);
  ToolPaths plan() const override final;

private:
  double plane_x_length_;
  double plane_y_length_;
  double z_offset_;
  double spacing_;
  geometry_msgs::Pose reference_frame_;
};

}  // namespace noether
