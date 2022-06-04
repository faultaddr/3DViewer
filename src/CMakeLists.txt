cmake_minimum_required(VERSION 3.5)

project(3DViewer)

# init qt
set(CMAKE_AUTOMOC ON) # for meta object compiler
set(CMAKE_AUTORCC ON) # resource files
set(CMAKE_AUTOUIC ON) # UI files

# find package
find_package(Qt5 REQUIRED Widgets)
find_package(PCL 1.12.1 REQUIRED)
add_subdirectory(log)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# find source files, header files and ui files
# aux_source_directory(. DIR_SRCS)
file(GLOB_RECURSE DIR_SRCS ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp)
file(GLOB_RECURSE DIR_HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
file(GLOB_RECURSE DIR_UIS ${CMAKE_CURRENT_SOURCE_DIR}/*.ui)

# add resource
set(RESOURCE ${CMAKE_CURRENT_SOURCE_DIR}/viewer.qrc)
qt5_add_resources(RESOURCE_ADDED ${RESOURCE})

add_executable(${PROJECT_NAME}
        ${DIR_SRCS}
        ${DIR_HEADERS}
        ${DIR_UIS}
        ${RESOURCE_ADDED})

target_link_libraries(${PROJECT_NAME}
        ${PCL_LIBRARIES}
        Qt5::Widgets
        glog::glog
        )
