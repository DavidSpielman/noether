#include <noether_tpp/tool_path_planners/raster/direction_generators/min_oobb_direction_generator.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/open3d_pcl_conversions.h>
#include <noether_tpp/utils.h>

#include <boost/make_shared.hpp>
#include <pcl/common/pca.h>

#include <open3d/geometry/BoundingVolume.h>
#include <open3d/geometry/TriangleMesh.h>

namespace noether
{

Eigen::Vector3d MinimumOOBBDirectionGenerator::generate(const pcl::PolygonMesh& mesh) const
{
  const open3d::geometry::TriangleMesh open3d_mesh = noether::pclMeshToOpen3dTriangleMesh(mesh);
  const open3d::geometry::OrientedBoundingBox oobb = open3d_mesh.GetMinimalOrientedBoundingBox(true);
  return oobb.R_.col(0);
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::MinimumOOBBDirectionGenerator>::encode(const noether::MinimumOOBBDirectionGenerator& val)
{
  return {};
}

bool convert<noether::MinimumOOBBDirectionGenerator>::decode(const Node& node,
                                                             noether::MinimumOOBBDirectionGenerator& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
