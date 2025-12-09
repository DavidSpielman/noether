#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup direction_generators
 * @brief Generates the raster direction along the largest principal axis of the minimum object oriented bounding box of
 * the input mesh
 */
class MinimumOOBBDirectionGenerator : public DirectionGenerator
{
public:
  MinimumOOBBDirectionGenerator(double rotation_offset = 0.0);
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

protected:
  /**
   * @brief Rotation offset (radians) to apply about the largest principal axis of the minimum object oriented bounding
   * box to the generated raster direction
   */
  double rotation_offset_;

  DECLARE_YAML_FRIEND_CLASSES(MinimumOOBBDirectionGenerator)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::MinimumOOBBDirectionGenerator)
