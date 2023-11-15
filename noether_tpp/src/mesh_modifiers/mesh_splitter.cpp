#include <noether_tpp/mesh_modifiers/mesh_splitter.h>
#include <noether_tpp/utils.h>
#include <pcl/PolygonMesh.h>
#include <noether_filtering/subset_extraction/subset_extractor.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

namespace noether
{
MeshSplitterModifier::MeshSplitterModifier(unsigned int filter_limit)
  : filter_limit_(filter_limit)
{
}

std::vector<pcl::PolygonMesh> MeshSplitterModifier::modify(const pcl::PolygonMesh& mesh) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<int> inlier_verticies;
  std::vector<int> outlier_verticies;

  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(0.0, filter_limit_);
  pass.filter(inlier_verticies);

  pass.setNegative(true);
  pass.filter(outlier_verticies);

  pcl::PolygonMesh inlier_mesh = extractSubMeshFromInlierVertices(mesh, inlier_verticies);
  pcl::PolygonMesh outlier_mesh = extractSubMeshFromInlierVertices(mesh, outlier_verticies);

  std::vector<pcl::PolygonMesh> meshes = {inlier_mesh, outlier_mesh};
  return meshes;
}

}  // namespace noether
