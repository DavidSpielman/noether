#include <noether_gui/widgets/tool_path_planners/raster/direction_generators/min_oobb_direction_generator_widget.h>
#include <noether_gui/widgets/angle_double_spin_box.h>
#include <noether_gui/utils.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

namespace noether
{
MinOOBBDirectionGeneratorWidget::MinOOBBDirectionGeneratorWidget(QWidget* parent)
  : BaseWidget(parent)
  , layout_(new QFormLayout(this))
  , label_(new QLabel("Rotation offset", this))
  , rotation_offset_(new AngleDoubleSpinBox(this))
{
  layout_->addRow(label_, rotation_offset_);
  rotation_offset_->setValue(0.0);
  rotation_offset_->setDecimals(3);
}

void MinOOBBDirectionGeneratorWidget::configure(const YAML::Node& config)
{
  auto rotation_offset = YAML::getMember<double>(config, "rotation_offset");
  rotation_offset_->setValue(rotation_offset);
}

void MinOOBBDirectionGeneratorWidget::save(YAML::Node& config) const
{
  config["name"] = "MinimumOOBB";
  config["rotation_offset"] = rotation_offset_->value();
}

}  // namespace noether
