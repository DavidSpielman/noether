#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/min_oobb_direction_generator_widget.h>

namespace noether
{
void MinOOBBDirectionGeneratorWidget::save(YAML::Node& config) const { config["name"] = "MinimumOOBB"; }

}  // namespace noether
