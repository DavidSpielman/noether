#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class MinOOBBDirectionGeneratorWidget : public BaseWidget
{
public:
  using BaseWidget::BaseWidget;
  using BaseWidget::configure;

  void save(YAML::Node&) const override;
};

}  // namespace noether
