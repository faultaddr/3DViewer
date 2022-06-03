#ifndef __FILE_IO_H__
#define __FILE_IO_H__

#include <QFileInfo>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>  // loadPolygonFileOBJ
#include <vector>
#include <string>
#include <map>
#include "data.h"

using std::vector;
using std::string;
using std::map;

class FileIO {
public:

    Data load(const QFileInfo& fileInfo);
    Data loadPLY(const QFileInfo& fileInfo);
    Data loadPCD(const QFileInfo& fileInfo);
    Data loadOBJ(const QFileInfo& fileInfo);
    Data loadSTL(const QFileInfo& fileInfo);
    Data loadVTK(const QFileInfo& fileInfo);

    bool save(const Data& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool savePLY(const Data& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool savePCD(const Data& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool saveOBJ(const Data& myCloud, const QFileInfo& fileInfo);
    bool saveSTL(const Data& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);
    bool saveVTK(const Data& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat);

    string getInputFormatsStr() const;
    string getOutputFormatsStr() const;

    map<string, string> inputFiltersMap = {
        {"ply", "Stanford Polygon File Format (*.ply)"},
        {"pcd", "PCL Point Cloud Data (*.pcd)"},
        {"obj", "Alias Wavefront Object (*.obj)"},
        {"stl", "STL File Format (*.stl)"},
        {"vtk", "Visualization Tookit Format (*.vtk)"},
        {"*", "All Files (*.*)"}
    };

    map<string, string> outputFiltersMap = {
        {"ply", "Stanford Polygon File Format (*.ply)"},
        {"pcd", "PCL Point Cloud Data (*.pcd)"},
        {"obj", "Alias Wavefront Object (*.obj)"},
        {"stl", "STL File Format (*.stl)"},
        {"vtk", "Visualization Tookit Format (*.vtk)"},
        {"*", "All Files (*.*)"}
    };
};

#endif
