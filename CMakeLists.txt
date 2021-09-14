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

find_package(catkin REQUIRED
  roscpp
  cmake_modules
  nodelet
  std_msgs
  mrs_lib
  visualization_msgs
  pcl_conversions
  dynamic_reconfigure
  )

generate_dynamic_reconfigure_options(
  cfg/octomap_editor.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options("-fPIC")
add_compile_options("${PCL_COMPILE_OPTIONS}")

find_package(octomap REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet std_msgs mrs_lib visualization_msgs
  LIBRARIES OctomapRvizVisualizer OctomapEditor OctomapSaver
  )

include_directories(
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
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
  )
