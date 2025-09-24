#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/publisher_handler.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

namespace mrs_octomap_tools
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
class OctomapRvizVisualizer : public rclcpp::Node
{
public:
  explicit OctomapRvizVisualizer(const rclcpp::NodeOptions & options);

private:
  bool is_initialized_ = false;

  mrs_lib::SubscriberHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  // normal markers


  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_occupied_marker_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_free_marker_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_throttled_occupied_marker_;
  mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray> pub_throttled_free_marker_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_occupied_pc_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2> pub_free_pc_;

  rclcpp::Time time_last_occupied_published_;
  rclcpp::Time time_last_free_published_;


  double         throttle_occupied_vis_ = 1;
  double         throttle_free_vis_     = 1;

  // Parameters
  double _occupancy_min_z_;
  double _occupancy_max_z_;

  static std_msgs::ColorRGBA heightMapColor(double h);
  bool                       getColor(typename OcTree_t::NodeType& node, std_msgs::ColorRGBA& color_out);

  double _occupancy_cube_size_factor_;
  double _free_cube_size_factor_;
  bool _use_colored_map_;
  std_msgs::msg::ColorRGBA _color_;
  std_msgs::msg::ColorRGBA _color_free_;
  double _color_factor_;
  bool _use_height_map_;
  bool _publish_free_space_;
  bool _filter_;
  bool _remove_ceiling_;

  // Methods
  void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  static std_msgs::msg::ColorRGBA heightMapColor(double h);
  bool getColor(typename OcTree_t::NodeType& node, std_msgs::msg::ColorRGBA& color_out);
};

template <typename OcTree_t>
OctomapRvizVisualizer<OcTree_t>::OctomapRvizVisualizer(const rclcpp::NodeOptions & options)
  : rclcpp::Node("octomap_rviz_visualizer", options)
{
  RCLCPP_INFO(this->get_logger(), "[OctomapRvizVisualizer]: initializing");

  mrs_lib::ParamLoader param_loader(this->shared_from_this(), "OctomapRvizVisualizer");

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
    RCLCPP_ERROR(this->get_logger(), "[OctomapRvizVisualizer]: could not load all parameters");
    rclcpp::shutdown();
    return;
  }

  // PublisherHandler options
  mrs_lib::PublisherHandlerOptions phopts;
  phopts.node = this->shared_from_this();

  pub_occupied_marker_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(phopts, "~/occupied_cells_vis_array_out");
  pub_free_marker_     = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(phopts, "~/free_cells_vis_array_out");
  pub_throttled_occupied_marker_ = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(phopts,"~/occupied_cells_vis_array_throttled_out");
  pub_throttled_free_marker_     = mrs_lib::PublisherHandler<visualization_msgs::msg::MarkerArray>(phopts,"~/free_cells_vis_array_throttled_out");
  pub_occupied_pc_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(phopts,"~/octomap_point_cloud_centers_out");
  pub_free_pc_     = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(phopts,"~/octomap_free_centers_out");


  time_last_occupied_published_ = rclcpp::Time(0);
  time_last_free_published_     = rclcpp::Time(0);


  // | ---------------------- check params ---------------------- |

  if (_use_height_map_ && _use_colored_map_) {
    std::string msg = std::string("You enabled both height map and RGBcolor registration.") +
                      " This is contradictory. " +
                      "Defaulting to height map.";
    RCLCPP_WARN(this->get_logger(), "[%s]: %s", this->get_name(), msg.c_str());
    _use_colored_map_ = false;
}
if (_use_colored_map_) {
    if (checkType<OcTree_t>("ColorOcTree")) {
        RCLCPP_WARN(this->get_logger(), "[%s]: Using RGB color registration (if information available)", this->get_name());
    } else {
        RCLCPP_WARN(this->get_logger(), "[%s]: Colored map requested, but this node does not support colors. Change octree_type in launch file.", this->get_name());
    }
}


  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscrirbeHandlerOptions shopts;
  shopts.node               = node_;
  shopts.node_name          = "OctomapRvizVisualizer";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe = true;
  shopts.autostart = true;

  auto callback = [this](const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
    this->callbackOctomap(msg);
  };

  sh_octomap_ = mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>(shopts, "~/octomap_in", callback);
  is_initialized_ = true;
  RCLCPP_INFO(this->get_logger(), "[OctomapRvizVisualizer]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOctomap() //{ */

template <typename OcTree_t>
void OctomapRvizVisualizer<OcTree_t>::callbackOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
  if (!is_initialized_) {
    return;
  }
  RCLCPP_INFO_ONCE(this->get_logger(), "[OctomapRvizVisualizer]: getting octomap");

  bool occupied_subscribed = pub_occupied_marker_->get_subscription_count() > 0;
  bool throttled_occupied_subscribed = pub_throttled_occupied_marker_->get_subscription_count() > 0;
  bool free_subscribed = pub_free_marker_->get_subscription_count() > 0;
  bool throttled_free_subscribed = pub_throttled_free_marker_->get_subscription_count() > 0;
  bool pc_occupied_subscribed = pub_occupied_pc_->get_subscription_count() > 0;
  bool pc_free_subscribed = pub_free_pc_->get_subscription_count() > 0;

  if (!occupied_subscribed && !throttled_occupied_subscribed && !free_subscribed && !throttled_free_subscribed &&
      !pc_occupied_subscribed && !pc_free_subscribed) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000, "[OctomapRvizVisualizer]: nobody subscribed to our messages, not processing octomap");
    return;
  }

  auto octomap = msg;
  if (!checkType<OcTree_t>(octomap->id)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *get_clock(), 2000, "Wrong octomap type. Change octree_type parameter.");
    return;
  }

  octomap::AbstractOcTree* tree_ptr;
  if (octomap->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!tree_ptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 1000, "[OctomapRvizVisualizer]: octomap message is empty!");
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

  // Initialize markers for free space
  visualization_msgs::msg::MarkerArray free_marker_array;
  const unsigned int tree_depth = octree->getTreeDepth();
  std::string world_frame = octomap->header.frame_id;
  free_marker_array.markers.resize(tree_depth + 1);

  // Initialize markers for occupied space
  visualization_msgs::msg::MarkerArray occupied_marker_array;
  occupied_marker_array.markers.resize(tree_depth + 1);

  // Initialize point clouds
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

  // Traverse all leafs in the tree
  for (auto it = octree->begin(tree_depth), end = octree->end(); it != end; ++it) {
    if (octree->isNodeOccupied(*it)) {
      if (!(occupied_subscribed || throttled_occupied_subscribed || pc_occupied_subscribed)) {
        continue;
      }
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > _occupancy_min_z_ && z - half_size < _occupancy_max_z_) {
        double x = it.getX();
        double y = it.getY();
        std_msgs::msg::ColorRGBA color;
        color.a = 1.0;
        bool got_color = getColor(*it, color);
        unsigned idx = it.getDepth();
        assert(idx < occupied_marker_array.markers.size());
        geometry_msgs::msg::Point cubeCenter;
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
          occupied_pcl_cloud.push_back(PCLPoint(float(x), float(y), float(z)));
        }
        }
      }
    } else {
      if (!(free_subscribed || throttled_free_subscribed || pc_free_subscribed)) {
        continue;
      }
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > _occupancy_min_z_ && z - half_size < _occupancy_max_z_) {
        if (free_subscribed || throttled_free_subscribed) {
          if (_publish_free_space_) {
            double x = it.getX();
            double y = it.getY();
            unsigned idx = it.getDepth();
            assert(idx < free_marker_array.markers.size());
            geometry_msgs::msg::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;
            free_marker_array.markers[idx].points.push_back(cubeCenter);
          }
        }
        if (pc_free_subscribed) {
          double x = it.getX();
          double y = it.getY();
          free_pcl_cloud.push_back(PCLPoint(float(x), float(y), float(z)));
        }
      }
    }
  }

  // Set all marker orientations
  for (int i = 0; i < tree_depth + 1; i++) {
    occupied_marker_array.markers[i].pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
    free_marker_array.markers[i].pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);
  }

  // Publish occupied markers
  if (occupied_subscribed || throttled_occupied_subscribed) {
    for (size_t i = 0; i < occupied_marker_array.markers.size(); ++i) {
      double size = octree->getNodeSize(i);
      occupied_marker_array.markers[i].header.frame_id = world_frame;
      occupied_marker_array.markers[i].header.stamp = now();
      occupied_marker_array.markers[i].ns = "map";
      occupied_marker_array.markers[i].id = i;
      occupied_marker_array.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      occupied_marker_array.markers[i].scale.x = size * _occupancy_cube_size_factor_;
      occupied_marker_array.markers[i].scale.y = size * _occupancy_cube_size_factor_;
      occupied_marker_array.markers[i].scale.z = size * _occupancy_cube_size_factor_;
      if (!_use_colored_map_) {
        occupied_marker_array.markers[i].color = _color_;
      }
      if (occupied_marker_array.markers[i].points.size() > 0) {
        occupied_marker_array.markers[i].action = visualization_msgs::msg::Marker::ADD;
      } else {
        occupied_marker_array.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      }
    }
    if (occupied_subscribed) {
      pub_occupied_marker_->publish(occupied_marker_array);
    }
    // Publish throttled
    if (throttled_occupied_subscribed) {
      const double last_pub_time = (now() - time_last_occupied_published_).seconds();
      const double max_time = 1.0 / throttle_occupied_vis_;
      if (last_pub_time >= max_time) {
        pub_throttled_occupied_marker_->publish(occupied_marker_array);
        time_last_occupied_published_ = now();
      }
    }
  }

  // Publish free markers
  if (free_subscribed || throttled_free_subscribed) {
    for (size_t i = 0; i < free_marker_array.markers.size(); ++i) {
      double size = octree->getNodeSize(i);
      free_marker_array.markers[i].header.frame_id = world_frame;
      free_marker_array.markers[i].header.stamp = now();
      free_marker_array.markers[i].ns = "map";
      free_marker_array.markers[i].id = i;
      free_marker_array.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      free_marker_array.markers[i].scale.x = size * _free_cube_size_factor_;
      free_marker_array.markers[i].scale.y = size * _free_cube_size_factor_;
      free_marker_array.markers[i].scale.z = size * _free_cube_size_factor_;
      free_marker_array.markers[i].color = _color_free_;
      if (free_marker_array.markers[i].points.size() > 0) {
        free_marker_array.markers[i].action = visualization_msgs::msg::Marker::ADD;
      } else {
        free_marker_array.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      }
    }
    if (free_subscribed) {
      pub_free_marker_->publish(free_marker_array);
    }
    // Publish throttled
    if (throttled_free_subscribed) {
      const double last_pub_time = (now() - time_last_free_published_).seconds();
      const double max_time = 1.0 / throttle_free_vis_;
      if (last_pub_time >= max_time) {
        pub_throttled_free_marker_->publish(free_marker_array);
        time_last_free_published_ = now();
      }
    }
  }

  // Publish occupied point cloud
  if (pc_occupied_subscribed) {
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(occupied_pcl_cloud, cloud);
    cloud.header.frame_id = world_frame;
    cloud.header.stamp = now();
    pub_occupied_pc_->publish(cloud);
  }

  // Publish free point cloud
  if (pc_free_subscribed) {
    sensor_msgs::msg::PointCloud2 cloud;
    pcl::toROSMsg(free_pcl_cloud, cloud);
    cloud.header.frame_id = world_frame;
    cloud.header.stamp = now();
    pub_free_pc_->publish(cloud);
  }
}

//}

// | ------------------------ routines ------------------------ |

/* heightMapColor() //{ */

template <typename OcTree_t>
std_msgs::msg::ColorRGBA OctomapRvizVisualizer<OcTree_t>::heightMapColor(double h) {
  std_msgs::msg::ColorRGBA color;
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
bool OctomapRvizVisualizer<OcTree_t>::getColor(typename OcTree_t::NodeType& node, std_msgs::msg::ColorRGBA& color_out) {
  /* RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "[Octomap_rviz_visualizer]: Cannot get color."); */
  return false;
}

template <>
bool OctomapRvizVisualizer<octomap::ColorOcTree>::getColor(octomap::ColorOcTree::NodeType& node, std_msgs::msg::ColorRGBA& color_out) {
  octomap::ColorOcTreeNode::Color& color = node.getColor();
  color_out.r = color.r / 255.;
  color_out.g = color.g / 255.;
  color_out.b = color.b / 255.;
  return true;
}
/*//}*/

}  // namespace octomap_rviz_visualizer
}  // namespace mrs_octomap_tools

// Register as a component
typedef mrs_octomap_tools::octomap_rviz_visualizer::OctomapRvizVisualizer<octomap::OcTree> OcTreeVisualizer;
typedef mrs_octomap_tools::octomap_rviz_visualizer::OctomapRvizVisualizer<octomap::ColorOcTree> ColorOcTreeVisualizer;
RCLCPP_COMPONENTS_REGISTER_NODE(OcTreeVisualizer)
RCLCPP_COMPONENTS_REGISTER_NODE(ColorOcTreeVisualizer)
