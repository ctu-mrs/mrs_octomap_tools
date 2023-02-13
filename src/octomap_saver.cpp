/* includes //{ */

#include "octomap/AbstractOcTree.h"
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
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

/* #ifdef COLOR_OCTOMAP_SERVER */
/* using OcTreeT = octomap::ColorOcTree; */
/* #else */
/* using OcTreeT = octomap::OcTree; */
/* #endif */

//}

/* class OctomapSaver //{ */

template <typename OcTreeT>
class OctomapSaver : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  std::string _map_path_;
  std::string _map_name_;
  bool        _binary_;

  mrs_lib::SubscribeHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);

  // | ------------------------ routines ------------------------ |

  bool saveToFile(std::shared_ptr<OcTreeT>& octree, const std::string& filename);

  ///Returns false, if the type_id (of the message) does not correspond to the template paramter
  ///of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

//}

/* onInit() //{ */

template <typename OcTreeT>
void OctomapSaver<OcTreeT>::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapSaver]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapSaver");

  param_loader.loadParam("map_path", _map_path_);
  param_loader.loadParam("map/name", _map_name_);
  param_loader.loadParam("binary", _binary_);

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

template <typename OcTreeT>
void OctomapSaver<OcTreeT>::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapSaver]: getting octomap");

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  if(!checkType(octomap->id)){
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
    ROS_WARN_THROTTLE(1.0, "[OctomapSaver]: octomap message is empty!");
    return;
  }

  std::shared_ptr<OcTreeT> octree = std::shared_ptr<OcTreeT>(dynamic_cast<OcTreeT*>(tree_ptr));

  saveToFile(octree, _map_name_);
}

//}

// | ------------------------ routines ------------------------ |

/* saveToFile() //{ */

template <typename OcTreeT>
bool OctomapSaver<OcTreeT>::saveToFile(std::shared_ptr<OcTreeT>& octree, const std::string& filename) {

  std::string ext = _binary_ ? ".bt" : ".ot";

  std::string file_path        = _map_path_ + "/" + filename + ext;
  std::string tmp_file_path    = _map_path_ + "/tmp_" + filename + ext;
  std::string backup_file_path = _map_path_ + "/" + filename + ext;

  try {
    std::filesystem::rename(file_path, backup_file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    ROS_ERROR("[OctomapEditor]: failed to copy map to the backup path");
  }

  std::string suffix = file_path.substr(file_path.length() - 3, 3);

  bool succ = false;

  if (_binary_) {
   succ = octree->writeBinary(tmp_file_path) ;
  } else {
   succ = octree->write(tmp_file_path) ;
  }

  if (!succ) {
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

template <typename OcTreeT>
bool OctomapSaver<OcTreeT>::checkType(std::string type_id)
{
  //General case: Need to be specialized for every used case
  /* setStatus(StatusProperty::Warn, "Messages", QString("Cannot verify octomap type")); */
  ROS_WARN_THROTTLE(2.0, "[Octomap_saver]: Cannot verify octomap type.");
  return true; //Try deserialization, might crash though
}
  
/* template <> */
/* bool OctomapSaver<octomap::OcTreeStamped>::checkType(std::string type_id) */
/* { */
/*   if(type_id == "OcTreeStamped") return true; */
/*   else return false; */
/* } */

template <>
bool OctomapSaver<octomap::OcTree>::checkType(std::string type_id)
{
  if(type_id == "OcTree") return true;
  else return false;
}

template <>
bool OctomapSaver<octomap::ColorOcTree>::checkType(std::string type_id)
{
  if(type_id == "ColorOcTree") return true;
  else return false;
}

}  // namespace octomap_saver

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>

typedef octomap_tools::octomap_saver::OctomapSaver<octomap::OcTree> OcTreeSaver;
typedef octomap_tools::octomap_saver::OctomapSaver<octomap::ColorOcTree> ColorOcTreeSaver;

/* PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_saver::OctomapSaver, nodelet::Nodelet) */
PLUGINLIB_EXPORT_CLASS(OcTreeSaver, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(ColorOcTreeSaver, nodelet::Nodelet)
