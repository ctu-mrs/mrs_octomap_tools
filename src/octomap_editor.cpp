/* includes //{ */

#include "octomap/OcTreeNode.h"
#include "octomap/Pointcloud.h"
#include "octomap/octomap_types.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/batch_visualizer.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/transform_broadcaster.h>

#include <visualization_msgs/MarkerArray.h>

#include <dynamic_reconfigure/server.h>
#include <octomap_tools/octomap_editorConfig.h>

#include <filesystem>

//}

namespace octomap_tools
{

namespace octomap_rviz_visualizer
{

/* class OctomapEditor //{ */

class OctomapEditor : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  std::string _map_path_;

  // | ----------------------- publishers ----------------------- |

  ros::Publisher pub_map_;

  // | --------------------- tf_broadcaster --------------------- |

  mrs_lib::TransformBroadcaster tf_broadcaster_;

  // | ------------------------- timers ------------------------- |

  ros::Timer timer_main_;
  void       timerMain([[maybe_unused]] const ros::TimerEvent& event);

  // | ------------------------- octomap ------------------------ |

  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex                       mutex_octree_;
  double                           octree_resolution_;

  // | ------------------------ routines ------------------------ |

  bool loadFromFile(const std::string& filename);
  bool saveToFile(const std::string& filename);
  void publishMap(void);
  void publishMarkers(void);

  void expandNodeRecursive(std::shared_ptr<octomap::OcTree>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);
  bool setInsideBBX(std::shared_ptr<octomap::OcTree>& from, const octomap::point3d& p_min, const octomap::point3d& p_max, const bool state);
  bool setUnknownInsideBBX(std::shared_ptr<octomap::OcTree>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool removeFreeInsideBBX(std::shared_ptr<octomap::OcTree>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool setUnknownToFreeInsideBBX(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);
  bool setFreeAboveGround(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const double height);

  // | -------------------- batch visualizer -------------------- |

  mrs_lib::BatchVisualizer bv_;
  ros::Publisher           pub_marker_texts_;

  // | --------------- dynamic reconfigure server --------------- |

  boost::recursive_mutex                           mutex_drs_;
  typedef octomap_tools::octomap_editorConfig      DrsParams_t;
  typedef dynamic_reconfigure::Server<DrsParams_t> Drs_t;
  boost::shared_ptr<Drs_t>                         drs_;
  void                                             callbackDrs(octomap_tools::octomap_editorConfig& params, uint32_t level);
  DrsParams_t                                      params_;
  std::mutex                                       mutex_params_;
};

//}

/* onInit() //{ */

void OctomapEditor::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapEditor]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapEditor");

  param_loader.loadParam("map_path", _map_path_);

  param_loader.loadParam("map/name", params_.map_name);

  param_loader.loadParam("roi_default/width", params_.roi_width);
  param_loader.loadParam("roi_default/depth", params_.roi_depth);
  param_loader.loadParam("roi_default/height", params_.roi_height);

  param_loader.loadParam("roi_default/x", params_.roi_x);
  param_loader.loadParam("roi_default/y", params_.roi_y);
  param_loader.loadParam("roi_default/z", params_.roi_z);

  param_loader.loadParam("roi_default/move_step", params_.roi_move_step);
  param_loader.loadParam("roi_default/resize_step", params_.roi_resize_step);

  param_loader.loadParam("free_above_ground_height", params_.free_above_ground_height);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapEditor]: could not load all parameters");
    ros::requestShutdown();
  }

  // | -------------------- batch visualizer -------------------- |

  bv_ = mrs_lib::BatchVisualizer(nh_, "markers", "map_frame");
  bv_.setPointsScale(0.5);
  bv_.setLinesScale(0.5);

  pub_marker_texts_ = nh_.advertise<visualization_msgs::MarkerArray>("marker_texts", 1);

  // | ----------------------- publishers ----------------------- |

  pub_map_ = nh_.advertise<octomap_msgs::Octomap>("octomap_out", 1);

  // | --------------------- tf boradcaster --------------------- |

  tf_broadcaster_ = mrs_lib::TransformBroadcaster();

  // | --------------------------- drs -------------------------- |

  params_.action_set_unknown           = false;
  params_.roi_fit_to_map               = false;
  params_.action_set_free              = false;
  params_.action_set_occupied          = false;
  params_.action_set_free_above_ground = false;
  params_.action_set_unknown_to_free   = false;
  params_.action_remove_free           = false;

  params_.action_save = false;
  params_.action_load = false;

  params_.roi_resize_plus_x  = false;
  params_.roi_resize_minus_x = false;

  params_.roi_resize_plus_y  = false;
  params_.roi_resize_minus_y = false;

  params_.roi_resize_plus_z  = false;
  params_.roi_resize_minus_z = false;

  params_.roi_move_plus_x  = false;
  params_.roi_move_minus_x = false;

  params_.roi_move_plus_y  = false;
  params_.roi_move_minus_y = false;

  params_.roi_move_plus_z  = false;
  params_.roi_move_minus_z = false;

  drs_.reset(new Drs_t(mutex_drs_, nh_));
  drs_->updateConfig(params_);
  Drs_t::CallbackType f = boost::bind(&OctomapEditor::callbackDrs, this, _1, _2);
  drs_->setCallback(f);

  // | ---------------------- load the map ---------------------- |

  if (loadFromFile(params_.map_name)) {
    ROS_INFO("[OctomapEditor]: map loaded");
  } else {
    ROS_ERROR("[OctomapEditor]: could not load the map");
  }

  // | ------------------------- timers ------------------------- |

  timer_main_ = nh_.createTimer(ros::Rate(1.0), &OctomapEditor::timerMain, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[OctomapEditor]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackDrs() //{ */

void OctomapEditor::callbackDrs(octomap_tools::octomap_editorConfig& params, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO("[OctomapEditor]: drs action");

  {
    std::scoped_lock lock(mutex_params_);

    params_ = params;
  }

  // | --------------------- update the ROI --------------------- |

  octomap::point3d roi_min(float(params_.roi_x - params_.roi_width), float(params_.roi_y - params_.roi_depth), float(params_.roi_z - params_.roi_height));
  octomap::point3d roi_max(float(params_.roi_x + params_.roi_width), float(params_.roi_y + params_.roi_depth), float(params_.roi_z + params_.roi_height));

  // | ------------------- process the actions ------------------ |

  /* load //{ */

  if (params.action_load) {

    if (loadFromFile(params.map_name)) {
      ROS_INFO("[OctomapEditor]: map loaded");
    } else {
      ROS_ERROR("[OctomapEditor]: could not load the map");
    }

    params.action_load = false;
    drs_->updateConfig(params);
  }

  //}

  /* save //{ */

  if (params.action_save) {

    if (saveToFile(params.map_name)) {
      ROS_INFO("[OctomapEditor]: map saved");
    } else {
      ROS_ERROR("[OctomapEditor]: could not save the map");
    }

    params.action_save = false;
    drs_->updateConfig(params);
  }

  //}

  /* set unkknown //{ */

  if (params.action_set_unknown) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting ROI unknown");

    setUnknownInsideBBX(octree_, roi_min, roi_max);

    params.action_set_unknown = false;
    drs_->updateConfig(params);
  }

  //}

  /* set free //{ */

  if (params.action_set_free) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting ROI free");

    setInsideBBX(octree_, roi_min, roi_max, false);

    params.action_set_free = false;
    drs_->updateConfig(params);
  }

  //}

  /* set unknown to free //{ */

  if (params.action_set_unknown_to_free) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting unknown in ROI to free");

    setUnknownToFreeInsideBBX(octree_, roi_min, roi_max);

    params.action_set_unknown_to_free = false;
    drs_->updateConfig(params);
  }

  //}

  /* remove free //{ */

  if (params.action_remove_free) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: remove free in ROI");

    removeFreeInsideBBX(octree_, roi_min, roi_max);

    params.action_remove_free = false;
    drs_->updateConfig(params);
  }

  //}

  /* set free above ground //{ */

  if (params.action_set_free_above_ground) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting ROI free above ground");

    setFreeAboveGround(octree_, roi_min, roi_max, params_.free_above_ground_height);

    params.action_set_free_above_ground = false;
    drs_->updateConfig(params);
  }

  //}

  /* set occupied //{ */

  if (params.action_set_occupied) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting ROI occupied");

    setInsideBBX(octree_, roi_min, roi_max, true);

    params.action_set_occupied = false;
    drs_->updateConfig(params);
  }

  //}

  /* roi movement //{ */

  if (params.roi_move_plus_x) {

    params.roi_x += params.roi_move_step;

    params.roi_move_plus_x = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_minus_x) {

    params.roi_x -= params.roi_move_step;

    params.roi_move_minus_x = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_plus_y) {

    params.roi_y += params.roi_move_step;

    params.roi_move_plus_y = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_minus_y) {

    params.roi_y -= params.roi_move_step;

    params.roi_move_minus_y = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_plus_z) {

    params.roi_z += params.roi_move_step;

    params.roi_move_plus_z = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_minus_z) {

    params.roi_z -= params.roi_move_step;

    params.roi_move_minus_z = false;
    drs_->updateConfig(params);
  }

  //}

  /* roi resizing //{ */

  if (params.roi_resize_plus_x) {

    params.roi_width += params.roi_resize_step;

    params.roi_resize_plus_x = false;
    drs_->updateConfig(params);
  }

  if (params.roi_resize_minus_x) {

    params.roi_width -= params.roi_resize_step;

    params.roi_resize_minus_x = false;
    drs_->updateConfig(params);
  }

  if (params.roi_resize_plus_y) {

    params.roi_depth += params.roi_resize_step;

    params.roi_resize_plus_y = false;
    drs_->updateConfig(params);
  }

  if (params.roi_resize_minus_y) {

    params.roi_depth -= params.roi_resize_step;

    params.roi_resize_minus_y = false;
    drs_->updateConfig(params);
  }

  if (params.roi_resize_plus_z) {

    params.roi_height += params.roi_resize_step;

    params.roi_resize_plus_z = false;
    drs_->updateConfig(params);
  }

  if (params.roi_move_minus_z) {

    params.roi_height -= params.roi_move_step;

    params.roi_move_minus_z = false;
    drs_->updateConfig(params);
  }

  //}

  /* roi fit to map //{ */

  if (params.roi_fit_to_map) {

    std::scoped_lock lock(mutex_octree_);

    ROS_INFO("[OctomapEditor]: setting ROI to fit the map");

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    params.roi_x = (min_x + max_x) / 2.0;
    params.roi_y = (min_y + max_y) / 2.0;
    params.roi_z = (min_z + max_z) / 2.0;

    params.roi_width  = fabs(min_x - max_x) / 2.0;
    params.roi_depth  = fabs(min_y - max_y) / 2.0;
    params.roi_height = fabs(min_z - max_z) / 2.0;

    params.roi_fit_to_map = false;
    drs_->updateConfig(params);
  }

  //}

  {
    std::scoped_lock lock(mutex_params_);

    params_ = params;
  }

  publishMarkers();
}

//}

// | ------------------------- timers ------------------------- |

/* timerMain() //{ */

void OctomapEditor::timerMain([[maybe_unused]] const ros::TimerEvent& evt) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_ONCE("[OctomapEditor]: main timer spinning");

  publishMap();

  publishMarkers();

  // | ------------------- publish the roi TF ------------------- |

  geometry_msgs::TransformStamped tf;
  tf.header.stamp            = ros::Time::now();
  tf.header.frame_id         = "map_frame";
  tf.child_frame_id          = "roi_frame";
  tf.transform.translation.x = params_.roi_x;
  tf.transform.translation.y = params_.roi_y;
  tf.transform.translation.z = params_.roi_z;
  tf.transform.rotation      = mrs_lib::AttitudeConverter(0, 0, 0);

  try {
    tf_broadcaster_.sendTransform(tf);
  }
  catch (...) {
    ROS_ERROR("[Odometry]: Exception caught during publishing TF: %s - %s.", tf.child_frame_id.c_str(), tf.header.frame_id.c_str());
  }
}

//}

// | ------------------------ routines ------------------------ |

/* loadFromFile() //{ */

bool OctomapEditor::loadFromFile(const std::string& filename) {

  std::string file_path = _map_path_ + "/" + filename + ".ot";

  {
    std::scoped_lock lock(mutex_octree_);

    if (file_path.length() <= 3)
      return false;

    std::string suffix = file_path.substr(file_path.length() - 3, 3);

    if (suffix == ".bt") {
      if (!octree_->readBinary(file_path)) {
        return false;
      }
    } else if (suffix == ".ot") {

      auto tree = octomap::AbstractOcTree::read(file_path);
      if (!tree) {
        return false;
      }

      octomap::OcTree* OcTree = dynamic_cast<octomap::OcTree*>(tree);
      octree_                 = std::shared_ptr<octomap::OcTree>(OcTree);

      if (!octree_) {
        ROS_ERROR("[OctomapEditor]: could not read OcTree file");
        return false;
      }

    } else {
      return false;
    }

    octree_resolution_ = octree_->getResolution();
  }

  return true;
}

//}

/* saveToFile() //{ */

bool OctomapEditor::saveToFile(const std::string& filename) {

  std::scoped_lock lock(mutex_octree_);

  std::string file_path        = _map_path_ + "/" + filename + ".ot";
  std::string tmp_file_path    = _map_path_ + "/tmp_" + filename + ".ot";
  std::string backup_file_path = _map_path_ + "/" + filename + "_backup.ot";

  try {
    std::filesystem::rename(file_path, backup_file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapEditor]: failed to copy map to the backup path");
  }

  std::string suffix = file_path.substr(file_path.length() - 3, 3);

  if (!octree_->write(tmp_file_path)) {
    ROS_ERROR("[OctomapEditor]: error writing to file '%s'", file_path.c_str());
    return false;
  }

  try {
    std::filesystem::rename(tmp_file_path, file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapEditor]: failed to copy map to the backup path");
  }

  return true;
}

//}

/* publishMap() //{ */

void OctomapEditor::publishMap(void) {

  std::scoped_lock lock(mutex_octree_);

  if (octree_) {
    octomap_msgs::Octomap map;
    map.header.frame_id = "map_frame";
    map.header.stamp    = ros::Time::now();

    if (octomap_msgs::fullMapToMsg(*octree_, map)) {
      pub_map_.publish(map);
    } else {
      ROS_ERROR("[OctomapServer]: error serializing local octomap to full representation");
    }
  }
}

//}

/* publishMarkers() //{ */

void OctomapEditor::publishMarkers(void) {

  auto params = mrs_lib::get_mutexed(mutex_params_, params_);

  bv_.clearBuffers();
  bv_.clearVisuals();

  // roi corners
  Eigen::Vector3d bot1(params.roi_x - params.roi_width, params.roi_y - params.roi_depth, params.roi_z - params.roi_height);
  Eigen::Vector3d bot2(params.roi_x + params.roi_width, params.roi_y - params.roi_depth, params.roi_z - params.roi_height);
  Eigen::Vector3d bot3(params.roi_x + params.roi_width, params.roi_y + params.roi_depth, params.roi_z - params.roi_height);
  Eigen::Vector3d bot4(params.roi_x - params.roi_width, params.roi_y + params.roi_depth, params.roi_z - params.roi_height);

  Eigen::Vector3d top1(params.roi_x - params.roi_width, params.roi_y - params.roi_depth, params.roi_z + params.roi_height);
  Eigen::Vector3d top2(params.roi_x + params.roi_width, params.roi_y - params.roi_depth, params.roi_z + params.roi_height);
  Eigen::Vector3d top3(params.roi_x + params.roi_width, params.roi_y + params.roi_depth, params.roi_z + params.roi_height);
  Eigen::Vector3d top4(params.roi_x - params.roi_width, params.roi_y + params.roi_depth, params.roi_z + params.roi_height);

  mrs_lib::geometry::Ray ray1 = mrs_lib::geometry::Ray(bot1, bot2);
  mrs_lib::geometry::Ray ray2 = mrs_lib::geometry::Ray(bot2, bot3);
  mrs_lib::geometry::Ray ray3 = mrs_lib::geometry::Ray(bot3, bot4);
  mrs_lib::geometry::Ray ray4 = mrs_lib::geometry::Ray(bot4, bot1);

  mrs_lib::geometry::Ray ray5 = mrs_lib::geometry::Ray(top1, top2);
  mrs_lib::geometry::Ray ray6 = mrs_lib::geometry::Ray(top2, top3);
  mrs_lib::geometry::Ray ray7 = mrs_lib::geometry::Ray(top3, top4);
  mrs_lib::geometry::Ray ray8 = mrs_lib::geometry::Ray(top4, top1);

  mrs_lib::geometry::Ray ray9  = mrs_lib::geometry::Ray(bot1, top1);
  mrs_lib::geometry::Ray ray10 = mrs_lib::geometry::Ray(bot2, top2);
  mrs_lib::geometry::Ray ray11 = mrs_lib::geometry::Ray(bot3, top3);
  mrs_lib::geometry::Ray ray12 = mrs_lib::geometry::Ray(bot4, top4);

  bv_.addRay(ray1, 1, 0, 0, 1.0);
  bv_.addRay(ray2, 1, 0, 0, 1.0);
  bv_.addRay(ray3, 1, 0, 0, 1.0);
  bv_.addRay(ray4, 1, 0, 0, 1.0);
  bv_.addRay(ray5, 1, 0, 0, 1.0);
  bv_.addRay(ray6, 1, 0, 0, 1.0);
  bv_.addRay(ray7, 1, 0, 0, 1.0);
  bv_.addRay(ray8, 1, 0, 0, 1.0);
  bv_.addRay(ray9, 1, 0, 0, 1.0);
  bv_.addRay(ray10, 1, 0, 0, 1.0);
  bv_.addRay(ray11, 1, 0, 0, 1.0);
  bv_.addRay(ray12, 1, 0, 0, 1.0);

  visualization_msgs::MarkerArray text_markers;
  int                             marker_id = 0;

  /* width //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top1.x() - top2.x()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "width";
    text_marker.pose.position.x = (top1.x() + top2.x()) / 2.0;
    text_marker.pose.position.y = (top1.y() + top2.y()) / 2.0;
    text_marker.pose.position.z = top1.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  /* depth //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top2.y() - top3.y()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "depth";
    text_marker.pose.position.x = (top2.x() + top3.x()) / 2.0;
    text_marker.pose.position.y = (top2.y() + top3.y()) / 2.0;
    text_marker.pose.position.z = top2.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  /* +X //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top2.y() - top3.y()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "+x";
    text_marker.pose.position.x = params.roi_x + 1.4 * (top2.x() - params.roi_x);
    text_marker.pose.position.y = (top2.y() + top3.y()) / 2.0;
    text_marker.pose.position.z = top1.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  /* -X //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top2.y() - top3.y()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "-x";
    text_marker.pose.position.x = params.roi_x + 1.4 * (top1.x() - params.roi_x);
    text_marker.pose.position.y = (top2.y() + top3.y()) / 2.0;
    text_marker.pose.position.z = top1.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  /* +Y //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top1.x() - top2.x()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "+y";
    text_marker.pose.position.x = (top3.x() + top4.x()) / 2.0;
    text_marker.pose.position.y = params.roi_y + 1.4 * (top3.y() - params.roi_y);
    text_marker.pose.position.z = top3.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  /* -Y //{ */

  {
    visualization_msgs::Marker text_marker;

    text_marker.header.frame_id = "map_frame";
    text_marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.color.a         = 1;
    text_marker.scale.z         = fabs(top1.x() - top2.x()) / 10;
    text_marker.color.r         = 1;
    text_marker.color.g         = 0;
    text_marker.color.b         = 0;

    text_marker.id = marker_id++;

    text_marker.text            = "-y";
    text_marker.pose.position.x = (top3.x() + top4.x()) / 2.0;
    text_marker.pose.position.y = params.roi_y + 1.4 * (top1.y() - params.roi_y);
    text_marker.pose.position.z = top3.z();

    text_marker.pose.orientation = mrs_lib::AttitudeConverter(0, 0, 0);

    text_markers.markers.push_back(text_marker);
  }

  //}

  pub_marker_texts_.publish(text_markers);

  bv_.publish();
}

//}

// | ---------------------- map functions --------------------- |

/* setInsideBBX() //{ */

bool OctomapEditor::setInsideBBX(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const bool state) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  auto coord_min = octree->keyToCoord(min_key);
  auto coord_max = octree->keyToCoord(max_key);

  for (int i = 0; i < int((coord_max.x() - coord_min.x()) / octree->getResolution()); i++) {

    double x = coord_min.x() + i * octree->getResolution();

    for (int j = 0; j < int((coord_max.y() - coord_min.y()) / octree->getResolution()); j++) {

      double y = coord_min.y() + j * octree->getResolution();

      for (int k = 0; k < int((coord_max.z() - coord_min.z()) / octree->getResolution()); k++) {

        double z = coord_min.z() + k * octree->getResolution();

        octomap::OcTreeKey key = octree->coordToKey(x, y, z);

        if (state) {
          octree->setNodeValue(key, 1.0);
        } else {
          octree->setNodeValue(key, -1.0);
        }
      }
    }
  }

  return true;
}

//}

/* setFreeAboveGround() //{ */

bool OctomapEditor::setFreeAboveGround(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max,
                                       const double height) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  auto coord_min = octree->keyToCoord(min_key);
  auto coord_max = octree->keyToCoord(max_key);

  for (int i = 0; i < int((coord_max.x() - coord_min.x()) / octree->getResolution()); i++) {

    double x = coord_min.x() + i * octree->getResolution();

    for (int j = 0; j < int((coord_max.y() - coord_min.y()) / octree->getResolution()); j++) {

      double y = coord_min.y() + j * octree->getResolution();

      bool got_ground   = false;
      int  ground_z_idx = 0;

      for (int k = int((coord_max.z() - coord_min.z()) / octree->getResolution()); k > 0; k--) {

        double z = coord_min.z() + k * octree->getResolution();

        octomap::OcTreeKey   key  = octree->coordToKey(x, y, z);
        octomap::OcTreeNode* node = octree->search(key);

        if (node && octree->isNodeOccupied(node)) {
          got_ground   = true;
          ground_z_idx = k;
          break;
        }
      }

      if (got_ground) {

        for (int k = ground_z_idx + 1; k < ground_z_idx + height; k++) {

          double z = coord_min.z() + k * octree->getResolution();

          octomap::OcTreeKey key = octree->coordToKey(x, y, z);

          octree->setNodeValue(key, -1.0);
        }
      }
    }
  }

  return true;
}

//}

/* setUnknownInsideBBX() //{ */

bool OctomapEditor::setUnknownInsideBBX(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octree->deleteNode(it.getKey());
  }

  return true;
}

//}

/* removeFreeInsideBBX() //{ */

bool OctomapEditor::removeFreeInsideBBX(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    auto key  = it.getKey();
    auto node = octree->search(key);

    if (node && !octree->isNodeOccupied(node)) {
      octree->deleteNode(key);
    }
  }

  return true;
}

//}

/* setUnknownToFreeInsideBBX() //{ */

bool OctomapEditor::setUnknownToFreeInsideBBX(std::shared_ptr<octomap::OcTree>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  auto coord_min = octree->keyToCoord(min_key);
  auto coord_max = octree->keyToCoord(max_key);

  for (int i = 0; i < int((coord_max.x() - coord_min.x()) / octree->getResolution()); i++) {

    double x = coord_min.x() + i * octree->getResolution();

    for (int j = 0; j < int((coord_max.y() - coord_min.y()) / octree->getResolution()); j++) {

      double y = coord_min.y() + j * octree->getResolution();

      for (int k = 0; k < int((coord_max.z() - coord_min.z()) / octree->getResolution()); k++) {

        double z = coord_min.z() + k * octree->getResolution();

        octomap::OcTreeKey key = octree->coordToKey(x, y, z);
        auto node = octree->search(key);

        if (!node) {
          octree->setNodeValue(key, -1.0);
        }
      }
    }
  }

  return true;
}

//}

/* expandNodeRecursive() //{ */

void OctomapEditor::expandNodeRecursive(std::shared_ptr<octomap::OcTree>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

  if (node_depth < octree->getTreeDepth()) {

    octree->expandNode(node);

    for (int i = 0; i < 8; i++) {
      auto child = octree->getNodeChild(node, i);

      expandNodeRecursive(octree, child, node_depth + 1);
    }

  } else {
    return;
  }
}

//}

}  // namespace octomap_rviz_visualizer

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_rviz_visualizer::OctomapEditor, nodelet::Nodelet)
