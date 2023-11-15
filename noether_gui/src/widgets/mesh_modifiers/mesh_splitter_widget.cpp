#include <noether_gui/widgets/mesh_modifiers/mesh_splitter_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/mesh_splitter.h>
#include <QFormLayout>
#include <QLabel>
#include <QDoubleSpinBox>

static const std::string filter_limit_KEY = "filter_limit";

namespace noether
{
MeshSplitterMeshModifierWidget::MeshSplitterMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent)
{
  auto layout = new QFormLayout(this);

  filter_limit_ = new QSpinBox(this);
  filter_limit_->setMinimum(0.0);
  filter_limit_->setSingleStep(1);
  filter_limit_->setValue(100);
  auto label_pnt = new QLabel("Filter Cuttoff", this);
  label_pnt->setToolTip("Cutoff for the filter to split mesh");
  layout->addRow(label_pnt, filter_limit_);

  setLayout(layout);
}

MeshModifier::ConstPtr MeshSplitterMeshModifierWidget::create() const
{
  return std::make_unique<MeshSplitterModifier>(
      filter_limit_->value());
}

void MeshSplitterMeshModifierWidget::configure(const YAML::Node& config)
{
  filter_limit_->setValue(getEntry<int>(config, filter_limit_KEY));
}

void MeshSplitterMeshModifierWidget::save(YAML::Node& config) const
{
  config[filter_limit_KEY] = filter_limit_->value();
}

}  // namespace noether
