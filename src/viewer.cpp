#include "viewer.h"
#include "glog/logging.h"

Viewer::Viewer(QWidget* parent) : QMainWindow(parent) {
  ui.setupUi(this);

  /***** Slots connection of QMenuBar and QToolBar *****/
  // File (connect)
  QObject::connect(ui.openAction, &QAction::triggered, this, &Viewer::open);
  QObject::connect(ui.addAction, &QAction::triggered, this, &Viewer::add);
  QObject::connect(ui.clearAction, &QAction::triggered, this, &Viewer::clear);

  ui.saveAction->setData(QVariant(false));       // isSaveBinary = false
  ui.saveBinaryAction->setData(QVariant(true));  // isSaveBinary = true
  connect(ui.saveAction, SIGNAL(triggered()), this, SLOT(save()));
  connect(ui.saveBinaryAction, SIGNAL(triggered()), this, SLOT(save()));
  QObject::connect(ui.exitAction, &QAction::triggered, this, &Viewer::exit);
  // Display (connect)
  QObject::connect(ui.pointcolorAction, &QAction::triggered, this,
                   &Viewer::pointcolorChanged);
  QObject::connect(ui.bgcolorAction, &QAction::triggered, this,
                   &Viewer::bgcolorChanged);
  QObject::connect(ui.mainviewAction, &QAction::triggered, this,
                   &Viewer::mainview);
  QObject::connect(ui.leftviewAction, &QAction::triggered, this,
                   &Viewer::leftview);
  QObject::connect(ui.topviewAction, &QAction::triggered, this,
                   &Viewer::topview);
  // Generate (connect)
  QObject::connect(ui.cubeAction, &QAction::triggered, this, &Viewer::cube);
  QObject::connect(ui.sphereAction, &QAction::triggered, this,
                   &Viewer::createSphere);
  QObject::connect(ui.cylinderAction, &QAction::triggered, this,
                   &Viewer::createCylinder);
  // Process (connect)
  QObject::connect(ui.meshsurfaceAction, &QAction::triggered, this,
                   &Viewer::convertSurface);
  QObject::connect(ui.wireframeAction, &QAction::triggered, this,
                   &Viewer::convertWireframe);
  // Option (connect)
  ui.windowsThemeAction->setData(QVariant(VIEWER_THEME_WINDOWS));
  ui.darculaThemeAction->setData(QVariant(VIEWER_THEME_DARCULA));
  ui.englishAction->setData(QVariant(VIEWER_LANG_ENGLISH));
  ui.chineseAction->setData(QVariant(VIEWER_LANG_CHINESE));
  connect(ui.windowsThemeAction, SIGNAL(triggered()), this,
          SLOT(ChangeTheme()));
  connect(ui.darculaThemeAction, SIGNAL(triggered()), this,
          SLOT(ChangeTheme()));
  connect(ui.englishAction, SIGNAL(triggered()), this, SLOT(ChangeLanguage()));
  connect(ui.chineseAction, SIGNAL(triggered()), this, SLOT(ChangeLanguage()));
  // About (connect)
  QObject::connect(ui.aboutAction, &QAction::triggered, this, &Viewer::about);
  QObject::connect(ui.helpAction, &QAction::triggered, this, &Viewer::help);

  /***** Slots connection of RGB widget *****/
  // Change size of cloud (connect)
  connect(ui.renderNum, SIGNAL(currentIndexChanged(int)), this,
          SLOT(RenderNumChanged(int)));
  connect(ui.pSlider, SIGNAL(valueChanged(int)), this,
          SLOT(PointSizeSliderChanged(int)));
  connect(ui.pSlider, SIGNAL(sliderReleased()), this,
          SLOT(PointSizeSliderReleased()));
  // Checkbox for coordinate and background color (connect)
  connect(ui.cooCbx, SIGNAL(stateChanged(int)), this, SLOT(CooCbxChecked(int)));
  connect(ui.bgcCbx, SIGNAL(stateChanged(int)), this, SLOT(BgcCbxChecked(int)));

  /***** Slots connection of dataTree(QTreeWidget) widget *****/
  // Item in dataTree is left-clicked (connect)

  connect(ui.dataTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this,
          SLOT(itemSelected(QTreeWidgetItem*, int)));
  // Item in dataTree is right-clicked
  connect(ui.dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this,
          SLOT(popMenu(const QPoint&)));

  connect(ui.dataTree, SIGNAL(itemChanged(QTreeWidgetItem*, int)), this,
          SLOT(TreeItemChanged(QTreeWidgetItem*, int)));

  connect(ui.consoleTable, SIGNAL(customContextMenuRequested(const QPoint&)),
          this, SLOT(popMenuInConsole(const QPoint&)));

  connect(ui.consoleTable, SIGNAL(customContextMenuRequested(const QPoint&)),
          this, SLOT(popMenuInConsole(const QPoint&)));

  // Initialization
  setAcceptDrops(true);
  initial();
}

Viewer::~Viewer() {}

void Viewer::dragEnterEvent(QDragEnterEvent* event) {
  LOG(INFO) << "dragEnterEvent";
  if (event->mimeData()->hasUrls()) {
    event->acceptProposedAction();
  } else if (event->mimeData()->hasText()) {
    LOG(INFO) << "dragEnterEvent from datalist";
    event->setDropAction(Qt::MoveAction);
    event->accept();
  } else {
    event->ignore();
  }
}

int Viewer::JudgeRender(int x, int y, int index) {
  // width: 695  height: 451
  auto renders = viewer->getRenderWindow()->GetRenderers();
  x = x - ui.dataTree->geometry().width() - ui.dataTree->x();
  for (int i = 0; i < renders->GetNumberOfItems(); i++) {
    auto render = static_cast<vtkRenderer*>(renders->GetItemAsObject(i));
    int count = 0;
    double vp[4];
    count += 1;
    render->GetViewport(vp);
    render->NormalizedDisplayToDisplay(vp[0], vp[1]);
    render->NormalizedDisplayToDisplay(vp[2], vp[3]);
    if (y >= vp[1] && y <= vp[3] && x >= vp[0] && x <= vp[2]) {
      render->Clear();
      viewer->addPointCloud(mycloud_vec[index].cloud,
                            mycloud_vec[index].cloudId, i);
      display_cloudId.push_back(mycloud_vec[index].cloudId);
      mycloud_vec[index].visible = true;
      ShowModel();
    }
  }

  return -1;
}
void Viewer::dropEvent(QDropEvent* event) {
  LOG(INFO) << "dropEvent";
  if (event->mimeData()->hasUrls())  //??????????????????
  {
    QList<QUrl> url_list = event->mimeData()->urls();
    QStringList file_path_list;
    for (auto& url : url_list) {
      if (url.isLocalFile()) {
        file_path_list.push_back(url.toLocalFile());
      }
    }
    doOpen(file_path_list);
  } else {
    // Judge the item which to render; set default to 0
    if (event->mimeData()->hasText()) {
      auto item = event->mimeData()->text();
      char* res = strtok(const_cast<char*>(item.toStdString().c_str()), "-");
      LOG(INFO) << res;
      vector<int> res_split;
      while (res != nullptr) {
        res_split.push_back(stoi(res));
        cout << res << endl;
        res = strtok(nullptr, "-");
      }

      int result =
          JudgeRender(event->pos().x() - ui.screen->pos().x(),
                      event->pos().y() - ui.screen->pos().y(), res_split[0]);
      if (result == -1) {
        event->ignore();
      } else {
      }
    }
  }
}

void Viewer::doOpen(const QStringList& filePathList) {
  // Open point cloud file one by one
  for (int i = 0; i != filePathList.size(); i++) {
    timeStart();                           // time start
    mycloud.cloud.reset(new PointCloudT);  // Reset cloud
    QFileInfo fileInfo(filePathList[i]);
    std::string filePath = fromQString(fileInfo.filePath());
    std::string fileName = fromQString(fileInfo.fileName());

    // begin loading
    ui.statusBar->showMessage(fileInfo.fileName() + ": " + QString::number(i) +
                              "/" + QString::number(filePathList.size()) +
                              " point cloud loading...");

    mycloud = fileIO.load(fileInfo);
    if (!mycloud.isValid) {
      // TODO: deal with the error, print error info in console?
      debug("invalid cloud.");
      LOG(ERROR) << "invalid cloud";
      continue;
    }
    // TODO: without sense, change later
    mycloud.viewer = viewer;
    mycloud_vec.push_back(mycloud);

    timeCostSecond = timeOff();  // time off

    consoleLog("Open", toQString(mycloud.fileName), toQString(mycloud.filePath),
               "Time cost: " + timeCostSecond + " s, Points: " +
                   QString::number(mycloud.cloud->points.size()));

    // update tree widget
    QTreeWidgetItem* cloudName =
        new QTreeWidgetItem(QStringList() << toQString(mycloud.fileName));
    auto icon = QIcon(":/Resources/images/icon.png");
    cloudName->setIcon(0, icon);
    cloudName->setCheckState(0, Qt::Checked);
    ui.dataTree->addTopLevelItem(cloudName);
    total_points += mycloud.cloud->points.size();
  }
  ui.statusBar->showMessage("");
  AddModel();
  setPropertyTable();
}

// Open point cloud
void Viewer::open() {
  QStringList filePathList = QFileDialog::getOpenFileNames(
      this, tr("Open point cloud file"), toQString(mycloud.fileDir),
      toQString(fileIO.getInputFormatsStr()));
  if (filePathList.isEmpty())
    return;

  // Clear cache
  // TODO: abstract a function
  mycloud_vec.clear();
  total_points = 0;
  ui.dataTree->clear();
  viewer->removeAllPointClouds();

  doOpen(filePathList);
}

// Add Point Cloud
void Viewer::add() {
  QStringList filePathList = QFileDialog::getOpenFileNames(
      this, tr("Add point cloud file"), toQString(mycloud.fileDir),
      toQString(fileIO.getInputFormatsStr()));
  if (filePathList.isEmpty())
    return;

  doOpen(filePathList);
}

// Clear all point clouds
void Viewer::clear() {
  mycloud_vec.clear();             //????????????????????????????????????
  viewer->removeAllPointClouds();  //???viewer?????????????????????
  viewer->removeAllShapes();       //??????remove?????????
  ui.dataTree->clear();            //???dataTree??????

  ui.propertyTable->clear();  //??????????????????propertyTable
  QStringList header;
  header << "Property"
         << "Value";
  ui.propertyTable->setHorizontalHeaderLabels(header);

  //????????????
  consoleLog("Clear", "All point clouds", "", "");

  setWindowTitle("3DViewer");  //??????????????????
  ShowModel();                 //????????????
}

// Save point cloud
void Viewer::save() {
  if (!mycloud.isValid) {
    QMessageBox::critical(this, tr("Saving file error"),
                          tr("There is no point cloud to save"));
    return;
  }

  // get binary flag from sender()
  QAction* action = qobject_cast<QAction*>(sender());
  QVariant v = action->data();
  bool isSaveBinary = (bool)v.value<bool>();

  QString selectedFilter =
      toQString(fileIO.outputFiltersMap.at(mycloud.fileSuffix));
  QString saveFilePath = QFileDialog::getSaveFileName(
      this,  // parent
      toQString("Save point cloud" +
                string(isSaveBinary ? " (binary)" : "")),  // caption
      toQString(mycloud.filePath),                         // dir
      toQString(fileIO.getOutputFormatsStr()),             // filter
      &selectedFilter                                      // selected filter
  );
  if (saveFilePath.isEmpty())
    return;

  QFileInfo fileInfo(saveFilePath);
  QString saveFileName = fileInfo.fileName();
  string saveFilePathStd = fromQString(saveFilePath);
  string saveFileNameStd = fromQString(saveFileName);

  if (mycloud_vec.size() > 1) {
    savemulti(fileInfo, isSaveBinary);
    return;
  }

  bool saveStatus = fileIO.save(mycloud, fileInfo, isSaveBinary);
  if (!saveStatus) {
    QMessageBox::critical(this, tr("Saving file error"),
                          tr("We can not save the file"));
    return;
  }

  consoleLog("Save", saveFileName, saveFilePath, "Single save");

  setWindowTitle(saveFilePath + " - 3DViewer");
  QMessageBox::information(
      this, tr("save point cloud file"),
      toQString("Save " + saveFileNameStd + " successfully!"));
}

// Save multi point cloud
void Viewer::savemulti(const QFileInfo& fileInfo, bool isSaveBinary) {
  string subname = fromQString(fileInfo.fileName());
  QString saveFilePath = fileInfo.filePath();
  PointCloudT::Ptr multi_cloud;
  multi_cloud.reset(new PointCloudT);
  multi_cloud->height = 1;
  int sum = 0;
  for (auto c : mycloud_vec) {
    sum += c.cloud->points.size();
  }
  multi_cloud->width = sum;
  multi_cloud->resize(multi_cloud->height * multi_cloud->width);
  int k = 0;
  for (int i = 0; i != mycloud_vec.size(); ++i) {
    // ??????cloudvec[i]->points.size()???cloudvec[i]->size()?????????
    for (int j = 0; j != mycloud_vec[i].cloud->points.size(); ++j) {
      multi_cloud->points[k].x = mycloud_vec[i].cloud->points[j].x;
      multi_cloud->points[k].y = mycloud_vec[i].cloud->points[j].y;
      multi_cloud->points[k].z = mycloud_vec[i].cloud->points[j].z;
      multi_cloud->points[k].r = mycloud_vec[i].cloud->points[j].r;
      multi_cloud->points[k].g = mycloud_vec[i].cloud->points[j].g;
      multi_cloud->points[k].b = mycloud_vec[i].cloud->points[j].b;
      k++;
    }
  }

  Data multiMyCloud;
  multiMyCloud.cloud = multi_cloud;
  multiMyCloud.isValid = true;

  // save multi_cloud
  bool saveStatus = fileIO.save(multiMyCloud, fileInfo, isSaveBinary);
  if (!saveStatus) {
    QMessageBox::critical(this, tr("Saving file error"),
                          tr("We can not save the file"));
    return;
  }

  if (isSaveBinary) {
    consoleLog("Save as binary", QString::fromLocal8Bit(subname.c_str()),
               saveFilePath, "Multi save (binary)");
  } else {
    consoleLog("Save", QString::fromLocal8Bit(subname.c_str()), saveFilePath,
               "Multi save");
  }

  // ??????????????? multi_cloud ??????????????? mycloud,????????????????????????????????????
  mycloud.cloud = multi_cloud;
  mycloud.filePath = fromQString(saveFilePath);
  mycloud.fileName = subname;

  setWindowTitle(saveFilePath + " - 3DViewer");
  QMessageBox::information(this, tr("save point cloud file"),
                           toQString("Save " + subname + " successfully!"));
}

//????????????
void Viewer::exit() {
  this->close();
}

// Generate cube
void Viewer::cube() {
  mycloud.cloud.reset(new PointCloudT);
  total_points = 0;
  ui.dataTree->clear();            //????????????????????????item
  viewer->removeAllPointClouds();  //???viewer?????????????????????
  mycloud_vec.clear();             //??????????????????

  mycloud.cloud->width = 50000;  // ???????????????
  mycloud.cloud->height = 1;  // ????????????????????????1???????????????????????????
  mycloud.cloud->is_dense = false;
  mycloud.cloud->resize(mycloud.cloud->width *
                        mycloud.cloud->height);  // ??????????????????
  for (size_t i = 0; i != mycloud.cloud->size(); ++i) {
    mycloud.cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    mycloud.cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    mycloud.cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
    mycloud.cloud->points[i].r = red;
    mycloud.cloud->points[i].g = green;
    mycloud.cloud->points[i].b = blue;
  }
  //?????????????????????
  QTreeWidgetItem* cloudName =
      new QTreeWidgetItem(QStringList() << QString::fromLocal8Bit("cube"));
  cloudName->setIcon(0, QIcon(":/Resources/images/icon.png"));
  ui.dataTree->addTopLevelItem(cloudName);

  // ????????????
  consoleLog("Generate cube", "cube", "cube", "");

  mycloud_vec.push_back(mycloud);
  AddModel();
}

//?????????
void Viewer::initial() {
  LOG(INFO) << "initial ";
  //???????????????
  setWindowIcon(QIcon(tr(":/Resources/images/icon.png")));
  setWindowTitle(tr("3DViewer"));

  //???????????????
  mycloud.cloud.reset(new PointCloudT);
  mycloud.cloud->resize(1);
  //	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
  auto renderer = vtkSmartPointer<vtkRenderer>::New();
  auto _renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  _renderWindow->AddRenderer(renderer);
  viewer.reset(new pcl::visualization::PCLVisualizer(renderer, _renderWindow,
                                                     "viewer", false));
  // viewer->addPointCloud(cloud, "cloud");
  ui.screen->setRenderWindow(viewer->getRenderWindow());
  viewer->setupInteractor(ui.screen->GetInteractor(),
                          ui.screen->GetRenderWindow());
  UpdateScreen();

  ui.propertyTable->setSelectionMode(
      QAbstractItemView::NoSelection);  // ?????????????????????????????? item
  ui.consoleTable->setSelectionMode(
      QAbstractItemView::NoSelection);  // ??????????????????????????? item
  ui.dataTree->setSelectionMode(
      QAbstractItemView::ExtendedSelection);  // ?????? dataTree ????????????

  // ??????????????????
  QString qss = darcula_qss;
  qApp->setStyleSheet(qss);

  setPropertyTable();
  setConsoleTable();

  // ????????????
  consoleLog("Software start", "viewer", "Welcome to use viewer", "faultaddr");

  // ????????????????????? dark
  viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
  window_to_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  window_to_image_filter->SetInput(ui.screen->renderWindow());
  ui.screen->setAcceptDrops(true);
  connect(ui.screen, SIGNAL(mouseEvent(QMouseEvent*)), this,
          SLOT(ReleaseMouseOnScreen(QMouseEvent*)));
}

//????????????
void Viewer::ShowModel() {
  LOG(INFO) << "ShowModel mycloud_vec size: " << mycloud_vec.size();
  bool changed = false;
  for (auto& my_cloud : mycloud_vec) {
    if (my_cloud.visible) {
      my_cloud.show();
      changed = true;
      viewer->updatePointCloud(my_cloud.cloud, my_cloud.cloudId);
    } else {
      my_cloud.hide();
    }
  }
  if (changed) {
    viewer->resetCamera();
  }
  UpdateScreen();
  CaptureModel();
}

//???????????????viewer,???????????????
void Viewer::AddModel(int view_port) {
  LOG(INFO) << "AddModel mycloud_vec size: " << mycloud_vec.size();
  for (int i = 0; i != mycloud_vec.size(); i++) {
    if (std::find(display_cloudId.begin(), display_cloudId.end(),
                  mycloud_vec[i].cloudId) == display_cloudId.end() &&
        mycloud_vec[i].visible) {
      LOG(INFO) << "update point cloud " << mycloud_vec[i].cloudId;
      viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId,
                            view_port);
      display_cloudId.push_back(mycloud_vec[i].cloudId);
      mycloud_vec[i].visible = true;
    } else {
      continue;
    }
  }
  ShowModel();
}

void Viewer::CaptureModel(int view_port) {
  LOG(INFO) << "CaptureModel";
  try {
    // screenshot code:
    window_to_image_filter->Modified();
    window_to_image_filter->Update();
    vtkImageData* id = window_to_image_filter->GetOutput();
    LOG(INFO) << "Win2Img done " << clock() * 1.0 / CLOCKS_PER_SEC << std::endl;
    vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
    writer->SetFileName("out.png");  // my image with borders 300x300 !!!!??
    writer->SetInputData(id);
    writer->Write();
  } catch (std::exception& e) {
    LOG(ERROR) << "exception: " << e.what();
  }
}
void Viewer::setCloudColor(unsigned int r, unsigned int g, unsigned int b) {
  // Set the new color
  for (size_t i = 0; i < mycloud.cloud->size(); i++) {
    mycloud.cloud->points[i].r = r;
    mycloud.cloud->points[i].g = g;
    mycloud.cloud->points[i].b = b;
    mycloud.cloud->points[i].a = 255;
  }
}

//??????
void Viewer::about() {
  AboutWin* aboutwin = new AboutWin(this);
  aboutwin->setModal(true);
  aboutwin->show();
  consoleLog("About", "faultaddr", "http://faultaddr.com",
             "Welcome to my blog!");
}

//??????
void Viewer::help() {
  QDesktopServices::openUrl(
      QUrl(QLatin1String("http://faultaddr.com/3DViewer")));
  consoleLog("Help", "3DViewer help", "http://faultaddr.com/3DViewer", "");
}

//??????????????????
void Viewer::createSphere() {
  mycloud.cloud.reset(new PointCloudT);
  ui.dataTree->clear();  //????????????????????????item
  viewer->removeAllShapes();
  mycloud_vec.clear();  //??????????????????

  pcl::PointXYZ p;
  p.x = 0;
  p.y = 0;
  p.z = 0;
  viewer->addSphere(p, 100, "sphere1");

  viewer->resetCamera();
  UpdateScreen();

  // ????????????
  consoleLog("Create sphere", "Sphere", "", "Succeeded");
}

void Viewer::createCylinder() {
  mycloud.cloud.reset(new PointCloudT);
  ui.dataTree->clear();  //????????????????????????item
  viewer->removeAllShapes();
  mycloud_vec.clear();  //??????????????????

  viewer->addCylinder(*(new pcl::ModelCoefficients()), "cylinder");

  viewer->resetCamera();
  UpdateScreen();

  // ????????????
  consoleLog("Create cylinder", "Cylinder", "", "Failed");
}

// Change theme: Windows/Darcula
void Viewer::ChangeTheme() {
  QAction* action = qobject_cast<QAction*>(sender());
  QVariant v = action->data();
  int theme = (int)v.value<int>();

  QColor colorLight(241, 241, 241, 255);
  QColor colorDark(0, 0, 0, 255);
  QString qss;

  switch (theme) {
    case VIEWER_THEME_WINDOWS: {
      qss = windows_qss;
      for (int i = 0; i != mycloud_vec.size(); i++) {
        if (ui.dataTree->topLevelItem(i)->textColor(0) == colorLight) {
          ui.dataTree->topLevelItem(i)->setTextColor(0, colorDark);
        }
      }
      theme_id = 0;
      consoleLog("Change theme", "Windows theme", "", "");
      break;
    }
    case VIEWER_THEME_DARCULA: {
      qss = darcula_qss;
      for (int i = 0; i != mycloud_vec.size(); i++) {
        if (ui.dataTree->topLevelItem(i)->textColor(0) == colorDark) {
          ui.dataTree->topLevelItem(i)->setTextColor(0, colorLight);
        }
      }
      consoleLog("Change theme", "Darcula theme", "", "");
      theme_id = 1;
      break;
    }
  }
  qApp->setStyleSheet(qss);
}

// Change language: English/Chinese
void Viewer::ChangeLanguage() {
  QAction* action = qobject_cast<QAction*>(sender());
  QVariant v = action->data();
  int language = (int)v.value<int>();

  switch (language) {
    case VIEWER_LANG_ENGLISH: {
      consoleLog("Change language", "English", "", "");
      break;
    }
    case VIEWER_LANG_CHINESE: {
      consoleLog("Change language", "Chinese",
                 "Doesn't support Chinese temporarily", "");
      break;
    }
  }
}

/*********************************************/
/*****************???????????????*****************/
/********************************************/

// //???????????????????????????
void Viewer::PointSizeSliderReleased() {
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  if (selected_item_count == 0) {
    for (int i = 0; i != mycloud_vec.size(); i++) {
      if (mycloud_vec[i].visible) {
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, p,
            mycloud_vec[i].cloudId);
      }
    }
    // ????????????
    consoleLog("Change cloud size", "All point clouds",
               "Size: " + QString::number(p), "");
  } else {
    for (int i = 0; i != selected_item_count; i++) {
      if (mycloud_vec[i].visible) {
        int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
        viewer->setPointCloudRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_POINT_SIZE, p,
            mycloud_vec[i].cloudId);
      }
    }
    // ????????????
    consoleLog("Change cloud size", "Point clouds selected",
               "Size: " + QString::number(p), "");
  }
  UpdateScreen();
}

void Viewer::PointSizeSliderChanged(int value) {
  p = value;
}

void Viewer::RenderNumChanged(int index) {
  LOG(INFO) << "RenderNumChanged";
  switch (index) {
    case 0: {
      break;
    }
    case 1: {
      int v1(0), v2(1);
      //??????????????? point
      viewer->removeAllPointClouds();
      viewer->getRenderWindow()->GetRenderers()->RemoveAllItems();
      viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
      viewer->addText("", 10, 10, "v1", v1);  //??????????????????
      viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
      viewer->addText("", 10, 10, "v2", v2);
      // ???????????? render window 1 ????????????????????????
      for (auto& model : mycloud_vec) {
        viewer->addPointCloud(model.cloud, model.cloudId, v1);
        viewer->updatePointCloud(model.cloud, model.cloudId);
      }
      UpdateScreen();
      // two render window
      break;
    }
    case 2: {
      // four render window
      ;
      break;
    }
    case 3: {
      // six render window
      ;
      break;
    }
  }
}
void Viewer::CooCbxChecked(int value) {
  switch (value) {
    case 0: {
      viewer->removeCoordinateSystem();
      consoleLog("Remove coordinate system", "Remove", "", "");
      break;
    }
    case 2: {
      viewer->addCoordinateSystem();
      consoleLog("Add coordinate system", "Add", "", "");
      break;
    }
  }
  UpdateScreen();
}

void Viewer::BgcCbxChecked(int value) {
  switch (value) {
    case 0: {
      viewer->setBackgroundColor(30 / 255.0, 30 / 255.0, 30 / 255.0);
      consoleLog("Change bg color", "Background", "30 30 30", "");
      break;
    }
    case 2: {
      //????????????setBackgroundColor()????????????0-1???double?????????
      viewer->setBackgroundColor(240 / 255.0, 240 / 255.0, 240 / 255.0);
      consoleLog("Change bg color", "Background", "240 240 240", "");
      break;
    }
  }
  UpdateScreen();
}

// ???????????????????????????????????????
void Viewer::pointcolorChanged() {
  QColor color =
      QColorDialog::getColor(Qt::white, this, "Select color for point cloud");

  if (color.isValid()) {
    // QAction* action = dynamic_cast<QAction*>(sender());
    // if (action != ui.pointcolorAction) //?????????????????????????????? dataTree
    QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
    int selected_item_count = ui.dataTree->selectedItems().size();
    if (selected_item_count == 0) {
      for (int i = 0; i != mycloud_vec.size(); ++i) {
        mycloud_vec[i].setPointColor(color.red(), color.green(), color.blue());
      }
      // ????????????
      consoleLog("Change cloud color", "All point clouds",
                 QString::number(color.red()) + " " +
                     QString::number(color.green()) + " " +
                     QString::number(color.blue()),
                 "");
    } else {
      for (int i = 0; i != selected_item_count; i++) {
        int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
        mycloud_vec[cloud_id].setPointColor(color.red(), color.green(),
                                            color.blue());
      }
      // ????????????
      consoleLog("Change cloud color", "Point clouds selected",
                 QString::number(color.red()) + " " +
                     QString::number(color.green()) + " " +
                     QString::number(color.blue()),
                 "");
    }
    ShowModel();
  }
}

//???????????????????????????????????????
void Viewer::bgcolorChanged() {
  QColor color =
      QColorDialog::getColor(Qt::white, this, "Select color for point cloud");
  if (color.isValid()) {
    viewer->setBackgroundColor(color.red() / 255.0, color.green() / 255.0,
                               color.blue() / 255.0);
    // ????????????
    consoleLog("Change bg color", "Background",
               QString::number(color.red()) + " " +
                   QString::number(color.green()) + " " +
                   QString::number(color.blue()),
               "");
    ShowModel();
  }
}

//?????????
void Viewer::mainview() {
  viewer->setCameraPosition(0, -1, 0, 0.5, 0.5, 0.5, 0, 0, 1);
  UpdateScreen();
}

void Viewer::leftview() {
  viewer->setCameraPosition(-1, 0, 0, 0, 0, 0, 0, 0, 1);
  UpdateScreen();
}

void Viewer::topview() {
  viewer->setCameraPosition(0, 0, 1, 0, 0, 0, 0, 1, 0);
  UpdateScreen();
}

// ????????????????????????
void Viewer::setPropertyTable() {
  QStringList header;
  header << "Property"
         << "Value";
  ui.propertyTable->setHorizontalHeaderLabels(header);
  ui.propertyTable->setItem(0, 0, new QTableWidgetItem("Clouds"));
  ui.propertyTable->setItem(
      0, 1, new QTableWidgetItem(QString::number(mycloud_vec.size())));

  ui.propertyTable->setItem(1, 0, new QTableWidgetItem("Points"));
  ui.propertyTable->setItem(1, 1, new QTableWidgetItem(""));

  ui.propertyTable->setItem(2, 0, new QTableWidgetItem("Faces"));
  ui.propertyTable->setItem(2, 1, new QTableWidgetItem(""));

  ui.propertyTable->setItem(3, 0, new QTableWidgetItem("Total points"));
  ui.propertyTable->setItem(
      3, 1, new QTableWidgetItem(QString::number(total_points)));

  ui.propertyTable->setItem(4, 0, new QTableWidgetItem("RGB"));
  ui.propertyTable->setItem(4, 1, new QTableWidgetItem(""));
}

void Viewer::setConsoleTable() {
  // ??????????????????
  QStringList header2;
  header2 << "Time"
          << "Operation"
          << "Operation object"
          << "Details"
          << "Note";
  ui.consoleTable->setHorizontalHeaderLabels(header2);
  ui.consoleTable->setColumnWidth(0, 150);
  ui.consoleTable->setColumnWidth(1, 200);
  ui.consoleTable->setColumnWidth(2, 200);
  ui.consoleTable->setColumnWidth(3, 300);

  // ui.consoleTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
  // //??????????????????
  ui.consoleTable->verticalHeader()->setDefaultSectionSize(22);  //????????????

  ui.consoleTable->setContextMenuPolicy(Qt::CustomContextMenu);
}

void Viewer::consoleLog(QString operation,
                        QString subname,
                        QString filename,
                        QString note) {
  if (enable_console == false) {
    return;
  }
  int rows = ui.consoleTable->rowCount();
  ui.consoleTable->setRowCount(++rows);
  QDateTime time = QDateTime::currentDateTime();  //???????????????????????????
  QString time_str = time.toString("MM-dd hh:mm:ss");  //??????????????????
  ui.consoleTable->setItem(rows - 1, 0, new QTableWidgetItem(time_str));
  ui.consoleTable->setItem(rows - 1, 1, new QTableWidgetItem(operation));
  ui.consoleTable->setItem(rows - 1, 2, new QTableWidgetItem(subname));
  ui.consoleTable->setItem(rows - 1, 3, new QTableWidgetItem(filename));
  ui.consoleTable->setItem(rows - 1, 4, new QTableWidgetItem(note));

  ui.consoleTable->scrollToBottom();  // ???????????????????????????
}

// QTreeWidget???item?????????????????????
void Viewer::itemSelected(QTreeWidgetItem* item, int count) {
  count = ui.dataTree->indexOfTopLevelItem(item);  //??????item?????????

  for (int i = 0; i != mycloud_vec.size(); i++) {
    if (mycloud_vec[i].visible) {
      viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1,
          mycloud_vec[i].cloudId);
    }
  }

  //?????????????????????RGB,?????????????????????
  int cloud_size = mycloud_vec[count].cloud->points.size();
  unsigned int cloud_r = mycloud_vec[count].cloud->points[0].r;
  unsigned int cloud_g = mycloud_vec[count].cloud->points[0].g;
  unsigned int cloud_b = mycloud_vec[count].cloud->points[0].b;
  bool multi_color = true;
  if (mycloud_vec[count].cloud->points.begin()->r ==
      (mycloud_vec[count].cloud->points.end() - 1)
          ->r)  //??????????????????????????????????????????????????????
    multi_color = false;

  ui.propertyTable->setItem(
      0, 1, new QTableWidgetItem(QString::number(mycloud_vec.size())));
  ui.propertyTable->setItem(1, 1,
                            new QTableWidgetItem(QString::number(cloud_size)));
  int faces = mycloud_vec[count].meshId.size() != 0
                  ? mycloud_vec[count].mesh->polygons.size()
                  : 0;
  ui.propertyTable->setItem(2, 1, new QTableWidgetItem(QString::number(faces)));
  ui.propertyTable->setItem(
      3, 1, new QTableWidgetItem(QString::number(total_points)));
  ui.propertyTable->setItem(
      4, 1,
      new QTableWidgetItem(multi_color ? "Multi Color"
                                       : (QString::number(cloud_r) + " " +
                                          QString::number(cloud_g) + " " +
                                          QString::number(cloud_b))));

  //??????item??????????????????????????????
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  int selected_item_count = ui.dataTree->selectedItems().size();
  for (int i = 0; i != selected_item_count; i++) {
    if (mycloud_vec[i].visible) {
      int cloud_id = ui.dataTree->indexOfTopLevelItem(itemList[i]);
      viewer->setPointCloudRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
          mycloud_vec[i].cloudId);
    }
  }
  // mycloud = mycloud_vec[count];
  UpdateScreen();
}

// consoleTable ??????????????????
void Viewer::popMenuInConsole(const QPoint&) {
  QAction clearConsoleAction("Clear console", this);
  QAction enableConsoleAction("Enable console", this);
  QAction disableConsoleAction("Disable console", this);

  connect(&clearConsoleAction, &QAction::triggered, this,
          &Viewer::clearConsole);
  connect(&enableConsoleAction, &QAction::triggered, this,
          &Viewer::enableConsole);
  connect(&disableConsoleAction, &QAction::triggered, this,
          &Viewer::disableConsole);

  QPoint pos;
  QMenu menu(ui.dataTree);
  menu.addAction(&clearConsoleAction);
  menu.addAction(&enableConsoleAction);
  menu.addAction(&disableConsoleAction);

  if (enable_console == true) {
    menu.actions()[1]->setVisible(false);
    menu.actions()[2]->setVisible(true);
  } else {
    menu.actions()[1]->setVisible(true);
    menu.actions()[2]->setVisible(false);
  }

  menu.exec(QCursor::pos());  //???????????????????????????
}

// ?????? consoleTable
void Viewer::clearConsole() {
  ui.consoleTable->clearContents();
  ui.consoleTable->setRowCount(0);
}

// ???????????? consoleTable
void Viewer::enableConsole() {
  enable_console = true;
}

// ?????? consoleTable
void Viewer::disableConsole() {
  clearConsole();
  enable_console = false;
}

// QTreeWidget???item?????????????????????
void Viewer::popMenu(const QPoint&) {
  QTreeWidgetItem* curItem = ui.dataTree->currentItem();  //??????????????????????????????
  if (curItem == NULL)
    return;  //????????????????????????????????????treeItem???????????????????????????????????????
  QString name = curItem->text(0);
  int id = ui.dataTree->indexOfTopLevelItem(curItem);
  Data& myCloud = mycloud_vec[id];

  QAction hideItemAction("Hide", this);
  QAction showItemAction("Show", this);
  QAction deleteItemAction("Delete", this);
  QAction changeColorAction("Change color", this);
  // show mode
  QAction pointModeAction("Set point mode", this);
  QAction meshModeAction("Set mesh mode", this);
  QAction pointMeshModeAction("Set point+mesh mode", this);
  pointModeAction.setData(QVariant(VIEWER_MODE_POINT));
  meshModeAction.setData(QVariant(VIEWER_MODE_MESH));
  pointMeshModeAction.setData(QVariant(VIEWER_MODE_POINT_MESH));

  pointModeAction.setCheckable(true);
  meshModeAction.setCheckable(true);
  pointMeshModeAction.setCheckable(true);

  if (myCloud.curMode == "point") {
    pointModeAction.setChecked(true);
  } else if (myCloud.curMode == "mesh") {
    meshModeAction.setChecked(true);
  } else if (myCloud.curMode == "point+mesh") {
    pointMeshModeAction.setChecked(true);
  }

  connect(&hideItemAction, &QAction::triggered, this, &Viewer::hideItem);
  connect(&showItemAction, &QAction::triggered, this, &Viewer::showItem);
  connect(&deleteItemAction, &QAction::triggered, this, &Viewer::deleteItem);
  connect(&changeColorAction, &QAction::triggered, this,
          &Viewer::pointcolorChanged);

  connect(&pointModeAction, SIGNAL(triggered()), this,
          SLOT(setRenderingMode()));
  connect(&meshModeAction, SIGNAL(triggered()), this, SLOT(setRenderingMode()));
  connect(&pointMeshModeAction, SIGNAL(triggered()), this,
          SLOT(setRenderingMode()));

  QPoint pos;
  QMenu menu(ui.dataTree);
  menu.addAction(&hideItemAction);
  menu.addAction(&showItemAction);
  menu.addAction(&deleteItemAction);
  menu.addAction(&changeColorAction);

  menu.addAction(&pointModeAction);
  menu.addAction(&meshModeAction);
  menu.addAction(&pointMeshModeAction);

  if (mycloud_vec[id].visible) {
    menu.actions()[1]->setVisible(false);
    menu.actions()[0]->setVisible(true);
  } else {
    menu.actions()[1]->setVisible(true);
    menu.actions()[0]->setVisible(false);
  }

  const vector<string> modes = myCloud.supportedModes;
  if (std::find(modes.begin(), modes.end(), "point") == modes.end()) {
    menu.actions()[4]->setVisible(false);
  }
  if (std::find(modes.begin(), modes.end(), "mesh") == modes.end()) {
    menu.actions()[5]->setVisible(false);
  }
  if (std::find(modes.begin(), modes.end(), "point+mesh") == modes.end()) {
    menu.actions()[6]->setVisible(false);
  }

  menu.exec(QCursor::pos());  //???????????????????????????
}

void Viewer::TreeItemChanged(QTreeWidgetItem* item, int index) {
  index = ui.dataTree->indexOfTopLevelItem(item);  //??????item?????????
  if (item->checkState(0)) {
    mycloud_vec[index].visible = true;
    HighLightTreeItemText(item);
  } else {
    mycloud_vec[index].visible = false;
    LowLightTreeItemText(item);
  }
  ShowModel();
}

void Viewer::HighLightTreeItemText(QTreeWidgetItem* item) {
  QColor item_color;
  if (theme_id == 0) {
    item_color = QColor(0, 0, 0, 255);
  } else {
    item_color = QColor(241, 241, 241, 255);
  }
  item->setTextColor(0, item_color);
}

void Viewer::LowLightTreeItemText(QTreeWidgetItem* item) {
  QColor item_color = QColor(112, 122, 132, 255);
  item->setTextColor(0, item_color);
}

void Viewer::hideItem() {
  LOG(INFO) << "hideItem";
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++) {
    // TODO hide?????????item???????????????????????????item?????????hideItem??? ????????????
    // ???showItem??? QTreeWidgetItem* curItem = ui.dataTree->currentItem();
    QTreeWidgetItem* item = itemList[i];
    QString name = item->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(item);
    mycloud_vec[id].hide();

    // QMessageBox::information(this, "cloud_id",
    // QString::fromLocal8Bit(cloud_id.c_str()));
    LowLightTreeItemText(item);
    mycloud_vec[id].visible = false;
    item->setCheckState(0, Qt::Unchecked);
  }

  // ????????????
  consoleLog("Hide point clouds", "Point clouds selected", "", "");
  viewer->resetCamera();
  UpdateScreen();  //?????????????????????????????????
}

void Viewer::showItem() {
  LOG(INFO) << "showItem";
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++) {
    // QTreeWidgetItem* curItem = ui.dataTree->currentItem();
    QTreeWidgetItem* item = itemList[i];
    QString name = item->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(item);
    // ???cloud_id?????????????????????????????????
    mycloud_vec[id].show();
    HighLightTreeItemText(item);

    mycloud_vec[id].visible = true;
    item->setCheckState(0, Qt::Checked);
  }

  // ????????????
  consoleLog("Show point clouds", "Point clouds selected", "", "");
  viewer->resetCamera();
  UpdateScreen();  //?????????????????????????????????
}

void Viewer::deleteItem() {
  LOG(INFO) << "deleteItem mycloud_vec size: " << mycloud_vec.size();
  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  // ui.dataTree->selectedItems().size()
  // ??????????????????????????????????????????????????????????????????????????? selected_item_count
  int selected_item_count = ui.dataTree->selectedItems().size();
  for (int i = 0; i != selected_item_count; i++) {
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    auto it = mycloud_vec.begin() + ui.dataTree->indexOfTopLevelItem(curItem);
    // ?????????????????????????????????????????????
    int delete_points = (*it).cloud->points.size();
    it = mycloud_vec.erase(it);

    total_points -= delete_points;
    setPropertyTable();

    ui.dataTree->takeTopLevelItem(ui.dataTree->indexOfTopLevelItem(curItem));
  }

  // ?????????????????????????????? id ??????????????????????????????????????????
  viewer->removeAllPointClouds();
  for (int i = 0; i != mycloud_vec.size(); i++) {
    viewer->addPointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
    viewer->updatePointCloud(mycloud_vec[i].cloud, mycloud_vec[i].cloudId);
  }

  // ????????????
  consoleLog("Delete point clouds", "Point clouds selected", "", "");

  UpdateScreen();
}

void Viewer::setRenderingMode() {
  QAction* action = qobject_cast<QAction*>(sender());
  QVariant v = action->data();
  int mode = (int)v.value<int>();
  string modeStr;

  switch (mode) {
    case VIEWER_MODE_POINT: {
      modeStr = "point";
      consoleLog("Point Mode", "Point clouds selected", "", "");
      break;
    }
    case VIEWER_MODE_MESH: {
      modeStr = "mesh";
      consoleLog("Mesh Mode", "Point clouds selected", "", "");
      break;
    }
    case VIEWER_MODE_POINT_MESH: {
      modeStr = "point+mesh";
      consoleLog("Point+Mesh Mode", "Point clouds selected", "", "");
      break;
    }
  }

  QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
  for (int i = 0; i != ui.dataTree->selectedItems().size(); i++) {
    QTreeWidgetItem* curItem = itemList[i];
    QString name = curItem->text(0);
    int id = ui.dataTree->indexOfTopLevelItem(curItem);
    Data& myCloud = mycloud_vec[id];
    myCloud.setShowMode(modeStr);
  }
  UpdateScreen();
}

int Viewer::convertSurface() {
  pcl::PointXYZ point;
  xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < mycloud.cloud->size(); i++) {
    point.x = mycloud.cloud->points[i].x;
    point.y = mycloud.cloud->points[i].y;
    point.z = mycloud.cloud->points[i].z;
    xyzCloud->push_back(point);
  }
  if (!xyzCloud) {
    return -1;
  }

  pcl::PolygonMesh mesh = triangulationGreedyProjection(xyzCloud);
  viewer->addPolygonMesh(mesh, "mesh-greedy-projection");
  viewer->setRepresentationToSurfaceForAllActors();

  consoleLog("Convert surface", "", "", "");

  viewer->removeAllShapes();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
  return 0;
}

int Viewer::convertWireframe() {
  pcl::PointXYZ point;
  xyzCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t i = 0; i < mycloud.cloud->size(); i++) {
    point.x = mycloud.cloud->points[i].x;
    point.y = mycloud.cloud->points[i].y;
    point.z = mycloud.cloud->points[i].z;
    xyzCloud->push_back(point);
  }
  if (!xyzCloud) {
    return -1;
  }

  pcl::PolygonMesh mesh = triangulationGreedyProjection(xyzCloud);
  viewer->addPolygonMesh(mesh, "mesh-greedy-projection");
  viewer->setRepresentationToWireframeForAllActors();

  consoleLog("Convert wire frame", "", "", "");

  viewer->removeAllShapes();
  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
  }
  return 0;
}

void Viewer::debug(const string& s) {
  QMessageBox::information(this, tr("Debug"),
                           QString::fromLocal8Bit(s.c_str()));
}

void Viewer::UpdateScreen() {
  ui.screen->update();
  ui.screen->renderWindow()->Render();
}
