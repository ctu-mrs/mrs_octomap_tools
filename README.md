# MRS Octomap Tools

This package contains a set of tools for OctoMap manipulation and visualization.

## OctomapMethods

A library with useful methods for manipulation and post-processing of OctoMap.

## OctomapSaver

A minimalistic node for saving an octomap from ROS topic.

## OctomapRvizVisualier

A publisher of RVIZ markers for OctoMap visualization.
It has some additional features over the original Octomap RVIZ plugin, e.g.:

* the option to filter the octomap,
* the option to remove ceiling from the octomap,
* throttled publishers for saving of power,
* power saving mode, when nobody subscribes to the markers.

## OctomapEditor

An editor for OctoMap. The editor uses RQT Reconfigure as a GUI tool and OctomapRvizVisualizer for real-time preview.
The editor provides adjustable bounding box to specify the region for operations.

Supported operations:
  * setting/unsetting free/occupied/unknown space within the bounding box or outside of the bounding box
  * changing of resolution of the octomap
  * creating free space above the ground level
  * dilation and erosion
  * expanding and pruning the tree and the bounding box region (mainly for testing)
  * undo (until your memory runs out)

![image](./.fig/octomap_editor.jpg)
