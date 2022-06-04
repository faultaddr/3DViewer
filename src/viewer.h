#ifndef VIEWER_H
#define VIEWER_H

// for solving error: no override found for 'vtkRenderWindow'
#include <vtkAutoInit.h>
// VTK_MODULE_INIT(vtkRenderingOpenGL2);
// VTK_MODULE_INIT(vtkInteractionStyle);

#include <QVTKOpenGLWidget.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/common/common.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkPNGWriter.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindow.h>
#include <vtkWindowToImageFilter.h>

#include <QAction>
#include <QColorDialog>
#include <QDebug>
#include <QDesktopServices>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QMouseEvent>
#include <QStatusBar>
#include <QString>
#include <QTextEdit>
#include <QTime>
#include <QToolBar>
#include <QUrl>
#include <QVBoxLayout>
#include <QtWidgets/QMainWindow>
#include <algorithm>
#include <map>
#include <vector>

#include "about_win.h"
#include "data.h"
#include "file_io.h"
#include "gbk.h"
#include "glog/logging.h"
#include "mesh_processing.h"
#include "tools.h"
#include "ui_viewer.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

const int VIEWER_THEME_WINDOWS = 0;
const int VIEWER_THEME_DARCULA = 1;

const int VIEWER_LANG_ENGLISH = 0;
const int VIEWER_LANG_CHINESE = 1;

const int VIEWER_MODE_POINT = 0;
const int VIEWER_MODE_MESH = 1;
const int VIEWER_MODE_POINT_MESH = 2;

using std::map;
using std::string;
using std::vector;

class Viewer : public QMainWindow {
  Q_OBJECT

 public:
  Viewer(QWidget* parent = 0);
  ~Viewer();

 private:
  Ui::Viewer ui;

  pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud;
  Data mycloud;
  std::vector<Data> mycloud_vec;
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

  FileIO fileIO;

  QString save_filename;
  long total_points = 0;  // Total amount of points in the viewer

  unsigned int red = 255;
  unsigned int green = 255;
  unsigned int blue = 255;
  unsigned int p = 2;
  std::vector<int> pointcolor;
  std::vector<int> bgcolor;

  QVBoxLayout* layout;

  int theme_id = 1;              // 0: Windows theme, 1: Darcula theme
  bool enable_console = true;    // console 的可用状态
  QString timeCostSecond = "0";  // 记录某个动作执行的时间
  vector<std::string> display_cloudId;
  vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter;
  /***** Slots of QMenuBar and QToolBar *****/
  // File menu slots
  void open();
  void add();
  void doOpen(const QStringList& filePathList);
  void clear();

  void savemulti(const QFileInfo& fileInfo, bool isSaveBinary);
  void exit();
  // Display menu slots
  void pointcolorChanged();
  void bgcolorChanged();
  void mainview();
  void leftview();
  void topview();
  // Generate menu slots
  void cube();
  void createSphere();
  void createCylinder();
  // Process menu slots
  int convertSurface();    //法线估计、曲面重建、网格面片显示
  int convertWireframe();  //法线估计、曲面重建、网格线框显示

  // About menu slots
  void about();
  void help();

  /***** Utils Methods ***/
  void initial();
  void ShowModel();                  //显示点云
  void AddModel(int view_port = 0);  //添加给viewer，显示点云
  void CaptureModel(int viwe_port = 0);

  void setCloudColor(unsigned int r, unsigned int g, unsigned int b);
  void HighLightTreeItemText(QTreeWidgetItem* item);
  void LowLightTreeItemText(QTreeWidgetItem* item);

  void setPropertyTable();

  void setConsoleTable();

  void consoleLog(QString operation, QString subname, QString filename,
                  QString note);

 public slots:
  void save();
  void ChangeTheme();
  void ChangeLanguage();

  // void colorBtnPressed();
  // void RGBsliderReleased();
  void PointSizeSliderReleased();
  void PointSizeSliderChanged(int value);
  // void rSliderChanged(int value);
  // void gSliderChanged(int value);
  // void bSliderChanged(int value);
  // Slots of checkBox
  void RenderNumChanged(int index);

  void CooCbxChecked(int value);
  void BgcCbxChecked(int value);

  /***** Slots of dataTree(QTreeWidget) widget *****/
  // Item in dataTree is left-clicked
  void itemSelected(QTreeWidgetItem*, int);
  // Item in dataTree is right-clicked
  void popMenu(const QPoint&);

  void TreeItemChanged(QTreeWidgetItem*, int);
  void hideItem();
  void showItem();
  void deleteItem();

  // set show mode
  void setRenderingMode();

  void popMenuInConsole(const QPoint&);
  void clearConsole();
  void enableConsole();
  void disableConsole();

  void debug(const string& s);
  void UpdateScreen();
};

#endif  // VIEWER_H
