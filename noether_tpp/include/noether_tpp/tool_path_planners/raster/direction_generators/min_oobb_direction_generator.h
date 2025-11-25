#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup direction_generators
 * @brief Generates the raster direction along the largest principal axis of the input mesh
 */
class MinimumOOBBDirectionGenerator : public DirectionGenerator
{
public:
  MinimumOOBBDirectionGenerator() = default;
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;

protected:
  DECLARE_YAML_FRIEND_CLASSES(MinimumOOBBDirectionGenerator)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::MinimumOOBBDirectionGenerator)
