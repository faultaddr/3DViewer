#include "data.h"

#include "pcl/conversions.h"

Data::Data() {
  cloud.reset(new PointCloudT);
  mesh.reset(new pcl::PolygonMesh);
}

Data::~Data() = default;

void Data::init(const QFileInfo &fileInfo, bool hasCloudParam,
                bool hasMeshParam) {
  hasCloud = hasCloudParam;
  hasMesh = hasMeshParam;

  filePath = fromQString(fileInfo.filePath());
  fileDir = fromQString(fileInfo.path());
  fileName = fromQString(fileInfo.fileName());
  fileSuffix = fromQString(fileInfo.suffix());

  if (!hasCloud && !hasMesh) {
    isValid = false;
    return;
  }

  isValid = true;
  if (hasMesh) {
    meshId = "mesh-" + fileName;
    cloudId = "cloud-" + fileName;
    pcl::fromPCLPointCloud2(mesh->cloud, *cloud);
    setPointAlpha(255);
    supportedModes = {"point", "mesh", "point+mesh"};
  }
  if (hasCloud) {
    cloudId = "cloud-" + fileName;
    setPointAlpha(255);
    supportedModes = {"point"};
  }

  // default show node
  curMode = "point";
}

void Data::setPointColor(int r, int g, int b) {
  for (auto &point : cloud->points) {
    point.r = r;
    point.g = g;
    point.b = b;
  }
}

void Data::setPointAlpha(int a) {
  for (auto &point : cloud->points) {
    point.a = a;
  }
}

void Data::setShowMode(const string &mode) {
  curMode = mode;
  show();
}

void Data::showCloud() {
  viewer->removePointCloud(cloudId);
  viewer->addPointCloud(cloud, cloudId);
  visible = true;
  //  viewer->setPointCloudRenderingProperties(
  //      pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 1.0,
  //      cloudId, 0);
}

void Data::hideCloud() {
  viewer->removePointCloud(cloudId);
  visible = false;
  //  viewer->setPointCloudRenderingProperties(
  //      pcl::visualization::RenderingProperties::PCL_VISUALIZER_OPACITY, 0.0,
  //      cloudId, 0);
}

void Data::showMesh() {
  if (meshId.empty()) return;  // no mesh
  visible = true;
  viewer->removePolygonMesh(meshId);
  viewer->addPolygonMesh(*mesh, meshId);
}

void Data::hideMesh() {
  if (meshId.empty()) return;
  visible = false;
  viewer->removePolygonMesh(meshId);
}

void Data::show() {
  if (curMode == "point") {
    hideMesh();
    showCloud();
  } else if (curMode == "mesh") {
    hideCloud();
    showMesh();
  } else if (curMode == "point+mesh") {
    showCloud();
    showMesh();
  }
}

void Data::hide() {
  hideCloud();
  hideMesh();
}
