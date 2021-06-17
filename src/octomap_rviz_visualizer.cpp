/* includes //{ */

#include "octomap/AbstractOcTree.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
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

//}

namespace octomap_rviz_visualizer
{

/* using //{ */

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT       = octomap::ColorOcTree;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTreeT       = octomap::OcTree;
#endif

//}

/* class OctomapRvizVisualizer //{ */

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

  double m_occupancyMinZ;
  double m_occupancyMaxZ;

  static std_msgs::ColorRGBA heightMapColor(double h);
  double                     _occupancy_cube_size_factor_;

  bool                m_useColoredMap;
  std_msgs::ColorRGBA m_color;
  std_msgs::ColorRGBA m_colorFree;
  double              m_colorFactor;

  bool m_useHeightMap;

  bool m_publishFreeSpace;
};

//}

/* onInit() //{ */

void OctomapRvizVisualizer::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapRvizVisualizer]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapRvizVisualizer");

  param_loader.loadParam("occupied_throttled_rate", throttle_occupied_vis_);
  param_loader.loadParam("free_throttled_rate", throttle_free_vis_);

  param_loader.loadParam("occupancy/min_z", m_occupancyMinZ);
  param_loader.loadParam("occupancy/max_z", m_occupancyMaxZ);
  param_loader.loadParam("occupancy/cube_size_factor", _occupancy_cube_size_factor_);

  param_loader.loadParam("colored_map/enabled", m_useColoredMap);

  param_loader.loadParam("height_map/enabled", m_useHeightMap);
  param_loader.loadParam("height_map/color_factor", m_colorFactor);

  double r, g, b, a;
  param_loader.loadParam("height_map/color/r", r);
  param_loader.loadParam("height_map/color/g", g);
  param_loader.loadParam("height_map/color/b", b);
  param_loader.loadParam("height_map/color/a", a);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  param_loader.loadParam("publish_free_space", m_publishFreeSpace, false);
  param_loader.loadParam("color_free/r", r);
  param_loader.loadParam("color_free/g", g);
  param_loader.loadParam("color_free/b", b);
  param_loader.loadParam("color_free/a", a);
  m_colorFree.r = r;
  m_colorFree.g = g;
  m_colorFree.b = b;
  m_colorFree.a = a;


  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapRvizVisualizer]: could not load all parameters");
    ros::requestShutdown();
  }

  // | ----------------------- publishers ----------------------- |

  // throttled
  pub_throttled_free_marker_     = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array_throttled_out", 1);
  pub_throttled_occupied_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array_throttled_out", 1);

  pub_occupied_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers_out", 1);
  pub_free_pc_     = nh_.advertise<sensor_msgs::PointCloud2>("octomap_free_centers_out", 1);

  time_last_occupied_published_ = ros::Time(0);
  time_last_free_published_     = ros::Time(0);

  // | ---------------------- check params ---------------------- |

  if (m_useHeightMap && m_useColoredMap) {
    std::string msg = std::string("You enabled both height map and RGBcolor registration.") + " This is contradictory. " + "Defaulting to height map.";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
    m_useColoredMap = false;
  }

  if (m_useColoredMap) {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_WARN("[%s]: Using RGB color registration (if information available)", ros::this_node::getName().c_str());
#else
    std::string msg = std::string("Colored map requested in launch file") + " - node not running/compiled to support colors, " +
                      "please define COLOR_OCTOMAP_SERVER and recompile or launch " + "the octomap_color_server node";
    ROS_WARN("[%s]: %s", ros::this_node::getName().c_str(), msg.c_str());
#endif
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapRvizVisualizer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 10;
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

void OctomapRvizVisualizer::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapRvizVisualizer]: getting octomap");

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  octomap::AbstractOcTree* tree_ptr = octomap_msgs::fullMsgToMap(*octomap);

  if (!tree_ptr) {
    ROS_WARN_THROTTLE(1.0, "[OctomapRvizVisualizer]: octomap message is empty!");
    return;
  }

  std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));

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
  pcl::PointCloud<PCLPoint> occupied_pclCloud;
  pcl::PointCloud<PCLPoint> free_pclCloud;

  // now, traverse all leafs in the tree:
  for (auto it = octree->begin(tree_depth), end = octree->end(); it != end; ++it) {

    if (octree->isNodeOccupied(*it)) {

      auto pes = it;

      double z2 = pes.getZ();

      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {

        double x = it.getX();
        double y = it.getY();

#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        unsigned idx = it.getDepth();
        assert(idx < occupied_marker_array.markers.size());

        geometry_msgs::Point cubeCenter;
        cubeCenter.x = x;
        cubeCenter.y = y;
        cubeCenter.z = z;

        occupied_marker_array.markers[idx].points.push_back(cubeCenter);

        if (m_useHeightMap) {

          double minX, minY, minZ, maxX, maxY, maxZ;
          octree->getMetricMin(minX, minY, minZ);
          octree->getMetricMax(maxX, maxY, maxZ);

          double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * m_colorFactor;
          occupied_marker_array.markers[idx].colors.push_back(heightMapColor(h));
        }

#ifdef COLOR_OCTOMAP_SERVER
        if (m_useColoredMap) {
          // TODO
          // potentially use occupancy as measure for alpha channel?
          std_msgs::msg::ColorRGBA _color;
          _color.r = (r / 255.);
          _color.g = (g / 255.);
          _color.b = (b / 255.);
          _color.a = 1.0;
          occupied_marker_array.markers[idx].colors.push_back(_color);
        }
#endif

#ifdef COLOR_OCTOMAP_SERVER
        PCLPoint _point = PCLPoint();
        _point.x        = x;
        _point.y        = y;
        _point.z        = z;
        _point.r        = r;
        _point.g        = g;
        _point.b        = b;
        occupied_pclCloud.push_back(_point);
#else
        occupied_pclCloud.push_back(PCLPoint(float(x), float(y), float(z)));
#endif
      }

    } else {

      // node not occupied => mark as free in 2D map if unknown so far
      double z         = it.getZ();
      double half_size = it.getSize() / 2.0;

      if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {

        if (m_publishFreeSpace) {

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

        // insert into pointcloud:
        double x = it.getX();
        double y = it.getY();

        free_pclCloud.push_back(PCLPoint(float(x), float(y), float(z)));
      }
    }
  }

  // set all the marker orietations
  for (int i = 0; i < tree_depth + 1; i++) {

    occupied_marker_array.markers[i].pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    free_marker_array.markers[i].pose.orientation     = mrs_lib::AttitudeConverter(0, 0, 0);
  }

  /* occupied marker //{ */

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

    if (!m_useColoredMap) {
      occupied_marker_array.markers[i].color = m_color;
    }

    if (occupied_marker_array.markers[i].points.size() > 0) {
      occupied_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      occupied_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }

  pub_occupied_marker_.publish(occupied_marker_array);

  // publisher throttled
  {
    const double last_pub_time = (ros::Time::now() - time_last_occupied_published_).toSec();
    const double max_time      = 1.0 / throttle_occupied_vis_;

    if (last_pub_time >= max_time) {
      pub_throttled_free_marker_.publish(occupied_marker_array);
      time_last_occupied_published_ = ros::Time::now();
    }
  }

  //}

  /* free marker //{ */

  for (size_t i = 0; i < free_marker_array.markers.size(); ++i) {

    double size = octree->getNodeSize(i);

    free_marker_array.markers[i].header.frame_id = world_frame;
    free_marker_array.markers[i].header.stamp    = ros::Time::now();
    free_marker_array.markers[i].ns              = "map";
    free_marker_array.markers[i].id              = i;
    free_marker_array.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
    free_marker_array.markers[i].scale.x         = size * _occupancy_cube_size_factor_;
    free_marker_array.markers[i].scale.y         = size * _occupancy_cube_size_factor_;
    free_marker_array.markers[i].scale.z         = size * _occupancy_cube_size_factor_;
    free_marker_array.markers[i].color           = m_colorFree;

    if (free_marker_array.markers[i].points.size() > 0)
      free_marker_array.markers[i].action = visualization_msgs::Marker::ADD;
    else
      free_marker_array.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  pub_free_marker_.publish(free_marker_array);

  // publisher throttled
  {
    const double last_pub_time = (ros::Time::now() - time_last_free_published_).toSec();
    const double max_time      = 1.0 / throttle_free_vis_;

    if (last_pub_time >= max_time) {
      pub_throttled_occupied_marker_.publish(free_marker_array);
      time_last_free_published_ = ros::Time::now();
    }
  }

  //}

  sensor_msgs::PointCloud2 cloud;
  // occupied
  pcl::toROSMsg(occupied_pclCloud, cloud);
  cloud.header.frame_id = world_frame;
  cloud.header.stamp    = ros::Time::now();
  pub_occupied_pc_.publish(cloud);

  // free
  pcl::toROSMsg(free_pclCloud, cloud);
  cloud.header.frame_id = world_frame;
  cloud.header.stamp    = ros::Time::now();
  pub_free_pc_.publish(cloud);
}

//}

// | ------------------------ routines ------------------------ |

/* heightMapColor() //{ */

std_msgs::ColorRGBA OctomapRvizVisualizer::heightMapColor(double h) {

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

}  // namespace octomap_rviz_visualizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_rviz_visualizer::OctomapRvizVisualizer, nodelet::Nodelet)
