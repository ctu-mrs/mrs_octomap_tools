/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <octomap_tools/octomap_methods.h>

//}

namespace octomap_tools
{

namespace octomap_rviz_visualizer
{

/* using //{ */

/* #ifdef COLOR_OCTOMAP_SERVER */
/* using PCLPoint      = pcl::PointXYZRGB; */
/* using PCLPointCloud = pcl::PointCloud<PCLPoint>; */
/* using OcTree_t       = octomap::ColorOcTree; */
/* #else */
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
/* using OcTree_t       = OcTree_t; */
/* #endif */

//}

/* class OctomapRvizVisualizer //{ */

template <typename OcTree_t>
class OctomapRvizVisualizer : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  mrs_lib::SubscribeHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);

  // normal markers

  ros::Publisher pub_occupied_marker_;
  ros::Publisher pub_free_marker_;

  // throttled markers
  ros::Publisher pub_throttled_occupied_marker_;
  ros::Publisher pub_throttled_free_marker_;
  ros::Time      time_last_occupied_published_;
  ros::Time      time_last_free_published_;
  double         throttle_occupied_vis_ = 1;
  double         throttle_free_vis_     = 1;

  // point cloud

  ros::Publisher pub_occupied_pc_;
  ros::Publisher pub_free_pc_;

  double _occupancy_min_z_;
  double _occupancy_max_z_;

  static std_msgs::ColorRGBA heightMapColor(double h);
  bool                       getColor(typename OcTree_t::NodeType& node, std_msgs::ColorRGBA& color_out);

  double _occupancy_cube_size_factor_;
  double _free_cube_size_factor_;

  bool                _use_colored_map_;
  std_msgs::ColorRGBA _color_;
  std_msgs::ColorRGBA _color_free_;
  double              _color_factor_;

  bool _use_height_map_;
  bool _publish_free_space_;

  bool _filter_;
  bool _remove_ceiling_;
};

//}

/* onInit() //{ */

template <typename OcTree_t>
void OctomapRvizVisualizer<OcTree_t>::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapRvizVisualizer]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapRvizVisualizer");

  param_loader.loadParam("occupied_throttled_rate", throttle_occupied_vis_);
  param_loader.loadParam("free_throttled_rate", throttle_free_vis_);

  param_loader.loadParam("occupied/min_z", _occupancy_min_z_);
  param_loader.loadParam("occupied/max_z", _occupancy_max_z_);
  param_loader.loadParam("occupied/cube_size_factor", _occupancy_cube_size_factor_);

  param_loader.loadParam("colored_map/enabled", _use_colored_map_);

  param_loader.loadParam("height_map/enabled", _use_height_map_);
  param_loader.loadParam("height_map/color_factor", _color_factor_);

  param_loader.loadParam("height_map/color/r", _color_.r);
  param_loader.loadParam("height_map/color/g", _color_.g);
  param_loader.loadParam("height_map/color/b", _color_.b);
  param_loader.loadParam("height_map/color/a", _color_.a);

  param_loader.loadParam("free_space/publish", _publish_free_space_);
  param_loader.loadParam("free_space/cube_size_factor", _free_cube_size_factor_);
  param_loader.loadParam("free_space/color/r", _color_free_.r);
  param_loader.loadParam("free_space/color/g", _color_free_.g);
  param_loader.loadParam("free_space/color/b", _color_free_.b);
  param_loader.loadParam("free_space/color/a", _color_free_.a);

  param_loader.loadParam("filter", _filter_);
  param_loader.loadParam("remove_ceiling", _remove_ceiling_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapRvizVisualizer]: could not load all parameters");
    ros::shutdown();
  }

  // | ----------------------- publishers ----------------------- |

  // throttled
  pub_throttled_occupied_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_throttled_out", 1);
  pub_throttled_free_marker_     = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array_throttled_out", 1);

  pub_occupied_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers_out", 1);
  pub_free_pc_     = nh_.advertise<sensor_msgs::PointCloud2>("octomap_free_centers_out", 1);

  time_last_occupied_published_ = ros::Time(0);
  time_last_free_published_     = ros::Time(0);

  // | ---------------------- check params ---------------------- |

  if (_use_height_map_ && _use_colored_map_) {
    std::string msg = std::string("You enabled both height map and RGBcolor registration.") + " This is contradictory. " + "Defaulting to height map.";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
    _use_colored_map_ = false;
  }

  if (_use_colored_map_) {
    if (checkType<OcTree_t>("ColorOcTree")) {
      ROS_WARN("[%s]: Using RGB color registration (if information available)", ros::this_node::getName().c_str());
    } else {
      ROS_WARN("[%s]: Colored map requested, but this node does not support colors. Change octree_type in launch file.", ros::this_node::getName().c_str());
    }
    /* #ifdef COLOR_OCTOMAP_SERVER */
    /*     ROS_WARN("[%s]: Using RGB color registration (if information available)", ros::this_node::getName().c_str()); */
    /* #else */
    /*     std::string msg = std::string("Colored map requested in launch file") + " - node not running/compiled to support colors, " + */
    /*                       "please define COLOR_OCTOMAP_SERVER and recompile or launch " + "the octomap_color_server node"; */
    /*     ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str()); */
    /* #endif */
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapRvizVisualizer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", &OctomapRvizVisualizer::callbackOctomap, this);

  pub_occupied_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_out", 1);
  pub_free_marker_     = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array_out", 1);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[OctomapRvizVisualizer]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOctomap() //{ */

template <typename OcTree_t>
void OctomapRvizVisualizer<OcTree_t>::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapRvizVisualizer]: getting octomap");

  bool occupied_subscribed          = pub_occupied_marker_.getNumSubscribers() > 0;
  bool throttled_occupied_subscibed = pub_throttled_occupied_marker_.getNumSubscribers() > 0;
  bool free_subscribed              = pub_free_marker_.getNumSubscribers() > 0;
  bool throttled_free_subscibed     = pub_throttled_free_marker_.getNumSubscribers() > 0;
  bool pc_occupied_subscribed       = pub_occupied_pc_.getNumSubscribers() > 0;
  bool pc_free_subscribed           = pub_free_pc_.getNumSubscribers() > 0;

  if (!occupied_subscribed && !throttled_occupied_subscibed && !free_subscribed && !throttled_free_subscibed && !pc_occupied_subscribed &&
      !pc_free_subscribed) {
    ROS_INFO_THROTTLE(1.0, "[OctomapRvizVisualizer]: nobody subscribed to our messages, not processing octomap");
    return;
  }

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  if (!checkType<OcTree_t>(octomap->id)) {
    ROS_ERROR_THROTTLE(2.0, "Wrong octomap type. Change octree_type parameter.");
    /* setStatusStd(StatusProperty::Error, "Message", "Wrong octomap type. Use a different display type."); */
    return;
  }

  octomap::AbstractOcTree* tree_ptr;

  if (octomap->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!tree_ptr) {
    ROS_WARN_THROTTLE(1.0, "[OctomapRvizVisualizer]: octomap message is empty!");
    return;
  }

  std::shared_ptr<OcTree_t> octree = std::shared_ptr<OcTree_t>(dynamic_cast<OcTree_t*>(tree_ptr));

  bool is_expanded = false;

  if (_filter_) {
    filterSpecs(octree, is_expanded, 2);
  }

  if (_remove_ceiling_) {
    removeCeiling(octree, is_expanded);
  }

  if (is_expanded) {
    octree->prune();
  }

  // init markers for free space:
  visualization_msgs::MarkerArray free_marker_array;

  const unsigned int tree_depth  = octree->getTreeDepth();
  std::string        world_frame = octomap->header.frame_id;

  // each array stores all cubes of a different size, one for each depth level:
  free_marker_array.markers.resize(tree_depth + 1);

  // init markers:
  visualization_msgs::MarkerArray occupied_marker_array;

  // each array stores all cubes of a different size, one for each depth level:
  occupied_marker_array.markers.resize(tree_depth + 1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> occupied_pcl_cloud;
  pcl::PointCloud<PCLPoint> free_pcl_cloud;

  double min_z_occupied = std::numeric_limits<double>::max();
  double max_z_occupied = std::numeric_limits<double>::lowest();

  for (auto it = octree->begin(), end = octree->end(); it != end; ++it) {

    if (octree->isNodeOccupied(*it)) {

      const double z = it.getZ();

      if (z < min_z_occupied) {
        min_z_occupied = z;
      }

      if (z > max_z_occupied) {
        max_z_occupied = z;
      }
    }
  }

  // now, traverse all leafs in the tree:
  for (auto it = octree->begin(tree_depth), end = octree->end(); it != end; ++it) {

    if (octree->isNodeOccupied(*it)) {

      if (!(occupied_subscribed || throttled_occupied_subscibed || pc_occupied_subscribed)) {
        continue;
      }

      double z2        = it.getZ();
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (occupied_subscribed || throttled_occupied_subscibed || pc_occupied_subscribed) {

        if (z + half_size > _occupancy_min_z_ && z - half_size < _occupancy_max_z_) {

          double x = it.getX();
          double y = it.getY();

          std_msgs::ColorRGBA color;
          color.a = 1.0;
          bool got_color = getColor(*it, color);

          unsigned idx = it.getDepth();
          assert(idx < occupied_marker_array.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupied_marker_array.markers[idx].points.push_back(cubeCenter);

          if (_use_height_map_) {

            double h = (1.0 - std::min(std::max((cubeCenter.z - min_z_occupied) / (max_z_occupied - min_z_occupied), 0.0), 1.0)) * _color_factor_;
            occupied_marker_array.markers[idx].colors.push_back(heightMapColor(h));
          }

          if (_use_colored_map_ && got_color) {
            occupied_marker_array.markers[idx].colors.push_back(color);
          }

          if (pc_occupied_subscribed) {
/* #ifdef COLOR_OCTOMAP_SERVER */
/*             PCLPoint _point = PCLPoint(); */
/*             _point.x        = x; */
/*             _point.y        = y; */
/*             _point.z        = z; */
/*             _point.r        = r; */
/*             _point.g        = g; */
/*             _point.b        = b; */
/*             occupied_pclCloud.push_back(_point); */
/* #else */
            occupied_pcl_cloud.push_back(PCLPoint(float(x), float(y), float(z)));
/* #endif */
          }
        }
      }

    } else {

      if (!(free_subscribed || throttled_free_subscibed || pc_free_subscribed)) {
        continue;
      }

      // node not occupied => mark as free in 2D map if unknown so far
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (z + half_size > _occupancy_min_z_ && z - half_size < _occupancy_max_z_) {

        if (free_subscribed || throttled_free_subscibed) {

          if (_publish_free_space_) {

            double x = it.getX();
            double y = it.getY();

            unsigned idx = it.getDepth();
            assert(idx < free_marker_array.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            free_marker_array.markers[idx].points.push_back(cubeCenter);
          }
        }

        if (pc_free_subscribed) {
          // insert into pointcloud:
          double x = it.getX();
          double y = it.getY();

          free_pcl_cloud.push_back(PCLPoint(float(x), float(y), float(z)));
        }
      }
    }
  }

  // set all the marker orietations
  // otherwise, rviz will object with a warning
  for (int i = 0; i < tree_depth + 1; i++) {

    occupied_marker_array.markers[i].pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    free_marker_array.markers[i].pose.orientation     = mrs_lib::AttitudeConverter(0, 0, 0);
  }

  /* occupied marker //{ */

  if (occupied_subscribed || throttled_occupied_subscibed) {

    for (size_t i = 0; i < occupied_marker_array.markers.size(); ++i) {

      double size = octree->getNodeSize(i);

      occupied_marker_array.markers[i].header.frame_id = world_frame;
      occupied_marker_array.markers[i].header.stamp    = ros::Time::now();
      occupied_marker_array.markers[i].ns              = "map";
      occupied_marker_array.markers[i].id              = i;
      occupied_marker_array.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
      occupied_marker_array.markers[i].scale.x         = size * _occupancy_cube_size_factor_;
      occupied_marker_array.markers[i].scale.y         = size * _occupancy_cube_size_factor_;
      occupied_marker_array.markers[i].scale.z         = size * _occupancy_cube_size_factor_;

      if (!_use_colored_map_) {
        occupied_marker_array.markers[i].color = _color_;
      }

      if (occupied_marker_array.markers[i].points.size() > 0) {
        occupied_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
      } else {
        occupied_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }

    if (occupied_subscribed) {
      pub_occupied_marker_.publish(occupied_marker_array);
    }

    // publisher throttled
    if (throttled_occupied_subscibed) {

      const double last_pub_time = (ros::Time::now() - time_last_occupied_published_).toSec();
      const double max_time      = 1.0 / throttle_occupied_vis_;

      if (last_pub_time >= max_time) {
        pub_throttled_occupied_marker_.publish(occupied_marker_array);
        time_last_occupied_published_ = ros::Time::now();
      }
    }
  }

  //}

  /* free marker //{ */

  if (free_subscribed || throttled_free_subscibed) {

    for (size_t i = 0; i < free_marker_array.markers.size(); ++i) {

      double size = octree->getNodeSize(i);

      free_marker_array.markers[i].header.frame_id = world_frame;
      free_marker_array.markers[i].header.stamp    = ros::Time::now();
      free_marker_array.markers[i].ns              = "map";
      free_marker_array.markers[i].id              = i;
      free_marker_array.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
      free_marker_array.markers[i].scale.x         = size * _free_cube_size_factor_;
      free_marker_array.markers[i].scale.y         = size * _free_cube_size_factor_;
      free_marker_array.markers[i].scale.z         = size * _free_cube_size_factor_;
      free_marker_array.markers[i].color           = _color_free_;

      if (free_marker_array.markers[i].points.size() > 0)
        free_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
      else
        free_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    if (free_subscribed) {
      pub_free_marker_.publish(free_marker_array);
    }

    // publisher throttled
    if (throttled_free_subscibed) {

      const double last_pub_time = (ros::Time::now() - time_last_free_published_).toSec();
      const double max_time      = 1.0 / throttle_free_vis_;

      if (last_pub_time >= max_time) {
        pub_throttled_free_marker_.publish(free_marker_array);
        time_last_free_published_ = ros::Time::now();
      }
    }
  }

  //}

  /* occupied pointcloud //{ */

  if (pc_occupied_subscribed) {
    sensor_msgs::PointCloud2 cloud;

    // occupied
    pcl::toROSMsg(occupied_pcl_cloud, cloud);
    cloud.header.frame_id = world_frame;
    cloud.header.stamp    = ros::Time::now();
    pub_occupied_pc_.publish(cloud);
  }

  //}

  /* free pointcloud //{ */

  if (pc_free_subscribed) {

    sensor_msgs::PointCloud2 cloud;

    pcl::toROSMsg(free_pcl_cloud, cloud);
    cloud.header.frame_id = world_frame;
    cloud.header.stamp    = ros::Time::now();
    pub_free_pc_.publish(cloud);
  }

  //}
}

//}

// | ------------------------ routines ------------------------ |

/* heightMapColor() //{ */

template <typename OcTree_t>
std_msgs::ColorRGBA OctomapRvizVisualizer<OcTree_t>::heightMapColor(double h) {

  std_msgs::ColorRGBA color;

  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int    i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }

  return color;
}

//}

/* getColor() */ /*//{*/
template <typename OcTree_t>
bool OctomapRvizVisualizer<OcTree_t>::getColor(typename OcTree_t::NodeType& node, std_msgs::ColorRGBA& color_out) {
  /* ROS_WARN_THROTTLE(2.0, "[Octomap_rviz_visualizer]: Cannot get color."); */
  return false;
}

template <>
bool OctomapRvizVisualizer<octomap::ColorOcTree>::getColor(octomap::ColorOcTree::NodeType& node, std_msgs::ColorRGBA& color_out) {
  octomap::ColorOcTreeNode::Color& color = node.getColor();
  color_out.r                            = color.r / 255.;
  color_out.g                            = color.g / 255.;
  color_out.b                            = color.b / 255.;
  return true;
}

/*//}*/

}  // namespace octomap_rviz_visualizer

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>

typedef octomap_tools::octomap_rviz_visualizer::OctomapRvizVisualizer<octomap::OcTree>      OcTreeVisualizer;
typedef octomap_tools::octomap_rviz_visualizer::OctomapRvizVisualizer<octomap::ColorOcTree> ColorOcTreeVisualizer;

PLUGINLIB_EXPORT_CLASS(OcTreeVisualizer, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ColorOcTreeVisualizer, nodelet::Nodelet)

/* PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_rviz_visualizer::OctomapRvizVisualizer, nodelet::Nodelet) */
