#pragma once

#include <noether_tpp/core/types.h>
#include <QWidget>
#include <QCheckBox>

class QVTKWidget;
class vtkActor;
class vtkPolyDataMapper;
class vtkProp;
class vtkRenderer;
class vtkAxes;
class vtkTubeFilter;

namespace boost_plugin_loader
{
class PluginLoader;
}

namespace Ui
{
class TPP;
}

namespace noether
{
class TPPPipelineWidget;

/**
 * @brief Basic tool path planning widget
 * @details Allows the user to laod a mesh from file, configure a tool path planning pipeline, and generate tool paths
 * from the input mesh
 */
class TPPWidget : public QWidget
{
  Q_OBJECT
public:
  TPPWidget(boost_plugin_loader::PluginLoader loader, QWidget* parent = nullptr);
  virtual ~TPPWidget();

  /**
   * @brief Get the planned tool paths
   * @details The highest level vector represents the tool paths generated for each mesh "fragment" generated by the
   * mesh modifier in the tool path planning pipeline
   */
  std::vector<ToolPaths> getToolPaths();

  void setMeshFile(const QString& file);
  void setConfigurationFile(const QString& file);

//private slots:
//  void onShowOriginalMesh(const bool /*checked*/);


private:
  void onLoadMesh(const bool /*checked*/);
  void onLoadConfiguration(const bool /*checked*/);
  void onSaveConfiguration(const bool /*checked*/);
  void onPlan(const bool /*checked*/);
  void onShowOriginalMesh();
  void onShowModifiedToolPath();


  Ui::TPP* ui_;
  TPPPipelineWidget* pipeline_widget_;

  // Viewer rendering
  QVTKWidget* render_widget_;
  vtkRenderer* renderer_;
  vtkPolyDataMapper* mesh_mapper_;
  vtkActor* mesh_actor_;
  std::vector<vtkProp*> tool_path_actors_;
  vtkAxes* axes_;
  vtkTubeFilter* tube_filter_;

  std::vector<ToolPaths> tool_paths_;
//  QPushButton *push_button_show_original_mesh;

};

}  // namespace noether
