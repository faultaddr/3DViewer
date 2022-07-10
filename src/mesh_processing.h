#pragma once

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

double calculatePCLPolygonMeshArea(const pcl::PolygonMesh&);

pcl::PolygonMesh triangulationGreedyProjection(
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyzCloud);
