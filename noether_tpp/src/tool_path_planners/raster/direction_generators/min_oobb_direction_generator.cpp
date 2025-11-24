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
MinimumOOBBDirectionGenerator::MinimumOOBBDirectionGenerator(double rotation_offset)
  : rotation_offset_(rotation_offset)
{
}

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
  Node node;
  node["rotation_offset"] = val.rotation_offset_;
  return node;
}

bool convert<noether::MinimumOOBBDirectionGenerator>::decode(const Node& node,
                                                               noether::MinimumOOBBDirectionGenerator& val)
{
  val.rotation_offset_ = getMember<double>(node, "rotation_offset");
  return true;
}
/** @endcond */

}  // namespace YAML
