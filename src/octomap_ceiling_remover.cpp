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

namespace octomap_ceiling_remover
{

/* using //{ */

#ifdef COLOR_OCTOMAP_SERVER
using OcTreeT = octomap::ColorOcTree;
#else
using OcTreeT = octomap::OcTree;
#endif

//}

/* class OctomapCeilingRemover //{ */

class OctomapCeilingRemover : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  ros::NodeHandle nh_;

  bool is_initialized_ = false;

  mrs_lib::SubscribeHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp);

  // | ------------------------ routines ------------------------ |

  bool saveToFile(std::shared_ptr<octomap::OcTree>& octree, const std::string& filename);
};

//}

/* onInit() //{ */

void OctomapCeilingRemover::onInit() {

  nh_ = nodelet::Nodelet::getMTPrivateNodeHandle();

  ros::Time::waitForValid();

  ROS_INFO("[OctomapCeilingRemover]: initializing");

  mrs_lib::ParamLoader param_loader(nh_, "OctomapCeilingRemover");

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[OctomapCeilingRemover]: could not load all parameters");
    ros::shutdown();
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscribeHandlerOptions shopts;
  shopts.nh                 = nh_;
  shopts.node_name          = "OctomapCeilingRemover";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;
  shopts.transport_hints    = ros::TransportHints().tcpNoDelay();

  sh_octomap_ = mrs_lib::SubscribeHandler<octomap_msgs::Octomap>(shopts, "octomap_in", &OctomapCeilingRemover::callbackOctomap, this);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  ROS_INFO("[OctomapCeilingRemover]: initialized");
}

//}

// | ------------------------ callbacks ----------------------- |

/* callbackOctomap() //{ */

void OctomapCeilingRemover::callbackOctomap(mrs_lib::SubscribeHandler<octomap_msgs::Octomap>& wrp) {

  if (!is_initialized_) {
    return;
  }

  ROS_INFO_THROTTLE(1.0, "[OctomapCeilingRemover]: getting octomap");

  octomap_msgs::OctomapConstPtr octomap = wrp.getMsg();

  octomap::AbstractOcTree* tree_ptr;

  if (octomap->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!tree_ptr) {
    ROS_WARN_THROTTLE(1.0, "[OctomapCeilingRemover]: octomap message is empty!");
    return;
  }

  std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));

  octree->expand();
}

//}

//}

}  // namespace octomap_ceiling_remover

}  // namespace octomap_tools

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_tools::octomap_ceiling_remover::OctomapCeilingRemover, nodelet::Nodelet)
