#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/tool_path_planners/flat_plane_toolpath_planner.h>
#include <geometry_msgs/msg/pose.h>

namespace noether
{
FlatPlaneToolPathPlanner::FlatPlaneToolPathPlanner(double plane_x_length, double plane_y_length, double z_offset, double spacing, geometry_msgs::Pose reference_frame)
  : plane_x_length_(plane_x_length), plane_y_length_(plane_y_length), z_offset_(z_offset), spacing_(spacing), reference_frame_(reference_frame)
{
}

ToolPaths FlatPlaneToolPathPlanner::plan() const
{
  // Convert geometry_msgs Pose to Eigen Isometry
  Eigen::Isometry3d eigen_pose = Eigen::Isometry3d::Identity();

  // Extract position from geometry_msgs Pose
  Eigen::Vector3d translation(reference_frame_.position.x, reference_frame_.position.y, reference_frame_.position.z);
  eigen_pose.translation() = translation;

  // Extract orientation from geometry_msgs Pose
  Eigen::Quaterniond quat(reference_frame_.orientation.w, reference_frame_.orientation.x,
                          reference_frame_.orientation.y, reference_frame_.orientation.z);
  eigen_pose.linear() = quat.toRotationMatrix();

  ToolPathSegment segment;
  ToolPath tool_path;
  ToolPaths tool_paths;
  double y_rotation_angle = 180;

  for(size_t i = 0; plane_x_length_ - spacing_*i >= 0; i++)
  {
    for(size_t j = 0; plane_y_length_ - spacing_*i >= 0; j++)
    {
      Eigen::Isometry3d pt = eigen_pose * Eigen::AngleAxisd(y_rotation_angle, Eigen::Vector3d::UnitY()) * Eigen::Translation3d(i*spacing_,j*spacing_,z_offset_);
      segment.push_back(pt);
    }
    if (i % 2 != 0)
      std::reverse(segment.begin(), segment.end());
  }
  tool_path = tool_path.push_back(segment);
  tool_paths = tool_paths.push_pack(tool_path);
  return tool_paths;
}
}  // namespace noether
