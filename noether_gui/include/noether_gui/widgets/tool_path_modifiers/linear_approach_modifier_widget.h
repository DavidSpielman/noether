#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class LinearApproachToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  LinearApproachToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
//  QDoubleSpinBox* offset_;
  Ui::Vector3dEditor* ui_;
  QSpinBox* n_points_;
};

}  // namespace noether
