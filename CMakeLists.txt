cmake_minimum_required(VERSION 3.1.2)
project(octomap_tools)

# Override CXX flags inherited from workspace, if precompiled PCL binaries from debian repos are used
if (DEFINED ENV{PCL_CROSS_COMPILATION})
  set(PCL_CROSS_COMPILATION $ENV{PCL_CROSS_COMPILATION})
else()
  set(PCL_CROSS_COMPILATION "false")
endif()
if(${PCL_CROSS_COMPILATION} STREQUAL "false")
  message("Using precompiled PCL binaries from debian repos. Overriding catkin workspace CMAKE_CXX_FLAGS.")
  set(CMAKE_CXX_FLAGS "-std=c++17")
else()
  message("Using custom-built PCL binaries. Inheriting all CMAKE_CXX_FLAGS from catkin workspace.")
endif()

add_definitions("-DOCTOMAP_NODEBUGOUT")

find_package(catkin REQUIRED
  roscpp
  cmake_modules
  nodelet
  std_msgs
  mrs_lib
  visualization_msgs
  pcl_conversions
  dynamic_reconfigure
  octomap_ros
  octomap_msgs
  )

generate_dynamic_reconfigure_options(
  cfg/octomap_editor.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-fPIC")
add_compile_options("${PCL_COMPILE_OPTIONS}")
add_compile_options("-O3")

find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp nodelet std_msgs mrs_lib visualization_msgs
  DEPENDS OCTOMAP
  LIBRARIES OctomapRvizVisualizer OctomapEditor OctomapSaver OctomapMethods
  )

include_directories(
  include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

# OctomapMethods

add_library(OctomapMethods
  src/octomap_methods.cpp
  )

add_dependencies(OctomapMethods
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapMethods
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# OctomapRvizVisualizer

add_library(OctomapRvizVisualizer
  src/octomap_rviz_visualizer.cpp
  )

add_dependencies(OctomapRvizVisualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapRvizVisualizer
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  OctomapMethods
  )

# OctomapEditor

add_library(OctomapEditor
  src/octomap_editor.cpp
  )

add_dependencies(OctomapEditor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapEditor
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  OctomapMethods
  )

# OctomapSaver

add_library(OctomapSaver
  src/octomap_saver.cpp
  )

add_dependencies(OctomapSaver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapSaver
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  OctomapMethods
  )
