cmake_minimum_required(VERSION 3.5)
project(mrs_octomap_tools)

# Set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
add_compile_options(-O3 -DOCTOMAP_NODEBUGOUT)
add_compile_options(-fPIC)
add_compile_options(${PCL_COMPILE_OPTIONS})

# Set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

if(ENABLE_COVERAGE)
  message(WARNING "Building for coverage, the performance might be limited")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")
  add_definitions("-O1")
  add_definitions("-g")
endif()

# Find dependencies
set(DEPENDENCIES
  ament_cmake
  rclcpp
  rclcpp_components
  std_msgs
  octomap_msgs
  octomap_ros
  pcl_conversions
  pcl_msgs
  pcl_ros
  visualization_msgs
  mrs_lib
)

set(LIBRARIES
  OctomapTools_OctomapRvizVisualizer
  OctomapTools_OctomapEditor
  OctomapTools_OctomapSaver
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# OctomapRvizVisualizer
add_library(OctomapTools_OctomapRvizVisualizer SHARED
  src/octomap_rviz_visualizer.cpp
)
ament_target_dependencies(OctomapTools_OctomapRvizVisualizer
  ${DEPENDENCIES}
)
target_link_libraries(OctomapTools_OctomapRvizVisualizer
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# OctomapEditor
add_library(OctomapTools_OctomapEditor SHARED
  src/octomap_editor.cpp
)
ament_target_dependencies(OctomapTools_OctomapEditor
  ${DEPENDENCIES}
)
target_link_libraries(OctomapTools_OctomapEditor
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# OctomapSaver
add_library(OctomapTools_OctomapSaver SHARED
  src/octomap_saver.cpp
)
ament_target_dependencies(OctomapTools_OctomapSaver
  ${DEPENDENCIES}
)
target_link_libraries(OctomapTools_OctomapSaver
  ${OCTOMAP_LIBRARIES}
  ${PCL_LIBRARIES}
)

# --------------------------------------------------------------
# |                           Install                          |
# --------------------------------------------------------------
ament_export_libraries(${LIBRARIES})
install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY tmux/
  USE_SOURCE_PERMISSIONS
  DESTINATION share/${PROJECT_NAME}/tmux
)

ament_export_include_directories(include)
ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  ${DEPENDENCIES}
)
ament_package()
