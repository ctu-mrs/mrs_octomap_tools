/* includes //{ */

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>

#include <filesystem>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//}

namespace octomap_tools
{

namespace octomap_saver
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

/* class OctomapSaver //{ */

class OctomapSaver : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  std::string _map_path_;
  std::string _map_name_;

  mrs_lib::SubscribeHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);

  // | ------------------------ routines ------------------------ |

  bool saveToFile(std::shared_ptr<octomap::OcTree> &octree, const std::string& filename);
};

//}

/* onInit() //{ */

void OctomapSaver::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapSaver]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapSaver");

  param_loader.loadParam("map_path", _map_path_);
  param_loader.loadParam("map/name", _map_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapSaver]: could not load all parameters");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapSaver";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", &OctomapSaver::callbackOctomap, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[OctomapSaver]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOctomap() //{ */

void OctomapSaver::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapSaver]: getting octomap");

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  octomap::AbstractOcTree* tree_ptr = octomap_msgs::fullMsgToMap(*octomap);

  if (!tree_ptr) {
    ROS_WARN_THROTTLE(1.0, "[OctomapSaver]: octomap message is empty!");
    return;
  }

  std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));

  saveToFile(octree, _map_name_);
}

//}

// | ------------------------ routines ------------------------ |

/* saveToFile() //{ */

bool OctomapSaver::saveToFile(std::shared_ptr<octomap::OcTree> &octree, const std::string& filename) {

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

  if (!octree->write(tmp_file_path)) {
    ROS_ERROR("[OctomapEditor]: error writing to file '%s'", file_path.c_str());
    return false;
  }

  try {
    std::filesystem::rename(tmp_file_path, file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapEditor]: failed to copy map to the backup path");
  }

  ROS_INFO("[OctomapSaver]: map saved");

  return true;
}

//}

}  // namespace octomap_saver

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_saver::OctomapSaver, nodelet::Nodelet)
