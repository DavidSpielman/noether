#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <pcl/PolygonMesh.h>
#include <vector>

namespace noether
{
/**
 * @brief Modifier that adjusts the parameters of the mesh splitter mesh modifier
 */
class MeshSplitterModifier : public MeshModifier
{
public:
  MeshSplitterModifier(unsigned int filter_limit);
  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  const unsigned int filter_limit_;
};

}  // namespace noether
