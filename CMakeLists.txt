cmake_minimum_required(VERSION 3.13)
project(mrs_octomap_tools)

add_definitions(-DOCTOMAP_NODEBUGOUT)
add_compile_options(-fPIC)
add_compile_options(${PCL_COMPILE_OPTIONS})

# disables anoying googletest warning
set(CATKIN_ENABLE_TESTING FALSE)

# disables anoying PCL warnings
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")

set(CATKIN_DEPENDENCIES
  cmake_modules
  dynamic_reconfigure
  mrs_lib
  nodelet
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_msgs
  roscpp
  std_msgs
  visualization_msgs
  )

set(LIBRARIES
  OctomapTools_OctomapRvizVisualizer
  OctomapTools_OctomapEditor
  OctomapTools_OctomapSaver
  )

find_package(catkin REQUIRED
  ${CATKIN_DEPENDENCIES}
  )

generate_dynamic_reconfigure_options(
  cfg/octomap_editor.cfg
  )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(octomap REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_DEPENDENCIES}
  DEPENDS OCTOMAP
  LIBRARIES ${LIBRARIES}
  )

include_directories(
  include
  ${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )

# OctomapRvizVisualizer

add_library(OctomapTools_OctomapRvizVisualizer
  src/octomap_rviz_visualizer.cpp
  )

add_dependencies(OctomapTools_OctomapRvizVisualizer
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapTools_OctomapRvizVisualizer
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# OctomapEditor

add_library(OctomapTools_OctomapEditor
  src/octomap_editor.cpp
  )

add_dependencies(OctomapTools_OctomapEditor
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapTools_OctomapEditor
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  )

# OctomapSaver

add_library(OctomapTools_OctomapSaver
  src/octomap_saver.cpp
  )

add_dependencies(OctomapTools_OctomapSaver
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  )

target_link_libraries(OctomapTools_OctomapSaver
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
  )

## --------------------------------------------------------------
## |                           Install                          |
## --------------------------------------------------------------

install(TARGETS ${LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tmux
  )

install(DIRECTORY include/mrs_octomap_tools/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )

install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
