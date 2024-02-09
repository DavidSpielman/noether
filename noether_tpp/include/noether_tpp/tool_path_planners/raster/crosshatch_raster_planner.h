#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
class CrosshatchRasterPlanner : public RasterPlanner
{
public:
CrosshatchRasterPlanner(std::vector<RasterPlanner::Ptr> planners)
  : RasterPlanner(nullptr, nullptr)
  , planners_(...)
{
}

ToolPaths create(const pcl::PolygonMesh& mesh) const override
{
  ToolPaths paths;
  for(const auto& planner : planners_)
  {
    ToolPaths tmp = planner.create(mesh);
    // Concatenate with previous
    paths.concatenate(tmp);
  }

  return paths;
}

protected:
  std::vector<RasterPlanner::Ptr> planners_;
};

}

// MOVE BELOW TO SNP TPP SIMPLE_RASTER_PLANER_WIDGET
namespace snp_tpp
{
class SimplePlaneSlicerCrosshatchRasterPlannerWidget : public RasterPlannerWidget
{
public:

  SimplePlaneSlicerCrosshatchRasterPlannerWidget(...)
    : RasterWidget(...)
    , widget_(new SimplePlaneSlicerRasterPlannerWidget(this, ...))
  {
  }

  ToolPathPlanner::Ptr create() override
  {
    std::vector<RasterPlanner::Ptr> planners;

    // Use the widget to create the first, "nominal" planner
    planners.push_back(widget_->create());

    // Create the cross-hatch planners
    // Get the nominal direction
    Eigen::Vector3d dir = widget->getDirectionGenerator->generate(mesh);

    // Get the reference rotation axis from the GUI or hard code
    Eigen::Vector3d real_rot_axis;
    {
      Eigen::Vector3d ref_rot_axis = Eigen::Vector3d::UnitZ(); // ui_->...

      // Rotate to get surface normal
      Eigen::Vector3d orthogonal_off_axis = ref_rot_axis.cross(dir);
      real_rot_axis = dir.cross(orthongonal_off_axis);
    }

    // Get the rotation angles from the GUI or hard code
    std::vector<double> rotation_angles = {M_PI / 2.0};  // ui_->...

    for(double angle : rotation_angles)
    {
      // Rotate the nominal direction about the real surface normal
      Eigen::Vector3d new_dir_axis = Eigen::AngleAxisd(angle, real_rot_axis) * dir;
      auto dir_gen = std::make_shared<FixedDirectionGenerator>(new_dir_axis);
      auto planner = std::make_shared<PlaneSliceRasterPlanner>(dir_gen, widget_->getOriginGenerator());

      planners.push_back(planner);
    }

    std::make_shared<CrosshatchRasterPlanner(planners);
  }

protected:
  SimplePlaneSlicerRasterPlannerWidget* widget_;
}

}
