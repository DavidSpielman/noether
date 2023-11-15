#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class MeshSplitterMeshModifierWidget : public MeshModifierWidget
{
  Q_OBJECT
public:
  MeshSplitterMeshModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QSpinBox* filter_limit_;
};

}  // namespace noether
