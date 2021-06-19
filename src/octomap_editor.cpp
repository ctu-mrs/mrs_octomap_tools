/* includes //{ */

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

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapEditor]: could not load all parameters");
    ros::requestShutdown();
  }

  // | ----------------------- publishers ----------------------- |

  pub_map_ = nh_.advertise<octomap_msgs::Octomap>("octomap_out", 1);

  // | --------------------------- drs -------------------------- |

  params_.action_clear = false;
  params_.action_save  = false;
  params_.action_load  = false;

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

  params_ = params;

  if (params.action_load) {

    if (loadFromFile(params.map_name)) {
      ROS_INFO("[OctomapEditor]: map loaded");
    } else {
      ROS_ERROR("[OctomapEditor]: could not load the map");
    }

    params.action_load = false;
    drs_->updateConfig(params);
  }

  if (params.action_save) {

    if (saveToFile(params.map_name)) {
      ROS_INFO("[OctomapEditor]: map saved");
    } else {
      ROS_ERROR("[OctomapEditor]: could not save the map");
    }

    params.action_save = false;
    drs_->updateConfig(params);
  }

  if (params.action_clear) {

    ROS_INFO("[OctomapEditor]: clearing the ROI");

    params.action_clear = false;
    drs_->updateConfig(params);
  }
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

      octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);
      octree_                 = std::shared_ptr<octomap::OcTree>(octree);

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

}  // namespace octomap_rviz_visualizer

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_rviz_visualizer::OctomapEditor, nodelet::Nodelet)
