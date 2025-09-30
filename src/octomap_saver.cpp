/* includes //{ */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscriber_handler.h>

#include <filesystem>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

//}

namespace mrs_octomap_tools
{

namespace octomap_saver
{

/* class OctomapSaver //{ */

template <typename OcTreeT>
class OctomapSaver : public rclcpp::Node {

public:
  explicit OctomapSaver(const rclcpp::NodeOptions & options);

private:
  bool is_initialized_ = false;

  // | ------------------------- params ------------------------- |

  std::string _map_path_;
  std::string _map_name_;
  bool        _binary_;

  mrs_lib::SubscriberHandler<octomap_msgs::Octomap> sh_octomap_;

  void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  // | ------------------------ routines ------------------------ |

  bool saveToFile(std::shared_ptr<OcTreeT>& octree, const std::string& filename);

  /// Returns false, if the type_id (of the message) does not correspond to the template paramter
  /// of this class, true if correct or unknown (i.e., no specialized method for that template).
  bool checkType(std::string type_id);
};

//}

/* constructor //{ */

template <typename OcTreeT>
OctomapSaver<OcTreeT>::OctomapSaver(const rclcpp::NodeOptions & options)
  : rclcpp::Node("octomap_saver", options)
{
  RCLCPP_INFO(this->get_logger(), "[OctomapSaver]: initializing");

  mrs_lib::ParamLoader param_loader(this->shared_from_this(), "OctomapSaver");

  param_loader.loadParam("map_path", _map_path_);
  param_loader.loadParam("map/name", _map_name_);
  param_loader.loadParam("binary", _binary_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(this->get_logger(), "[OctomapSaver]: could not load all parameters");
    rclcpp::shutdown();
    return;
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = this->shared_from_this();
  shopts.node_name          = "OctomapSaver";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;

  auto callback = [this](const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
    this->callbackOctomap(msg);
  };

  sh_octomap_ = mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>(shopts, "~/octomap_in", callback);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "[OctomapSaver]: initialized");
}


// | ------------------------ callbacks ----------------------- |

/* callbackOctomap() //{ */

template <typename OcTreeT>
void OctomapSaver<OcTreeT>::callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[OctomapSaver]: getting octomap");

  auto octomap = msg;

  if (!checkType(octomap->id)) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Wrong octomap type. Change octree_type parameter.");
    return;
  }

  octomap::AbstractOcTree* tree_ptr;

  if (octomap->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!tree_ptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[OctomapSaver]: octomap message is empty!");
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
    RCLCPP_ERROR(this->get_logger(), "[OctomapEditor]: failed to copy map to the backup path");
  }

  std::string suffix = file_path.substr(file_path.length() - 3, 3);

  bool succ = false;

  if (_binary_) {
    succ = octree->writeBinary(tmp_file_path);
  } else {
    succ = octree->write(tmp_file_path);
  }

  if (!succ) {
    RCLCPP_ERROR(this->get_logger(), "[OctomapEditor]: error writing to file '%s'", file_path.c_str());
    return false;
  }

  try {
    std::filesystem::rename(tmp_file_path, file_path);
  }
  catch (std::filesystem::filesystem_error& e) {
    RCLCPP_ERROR(this->get_logger(), "[OctomapEditor]: failed to copy map to the backup path");
  }

  RCLCPP_INFO(this->get_logger(), "[OctomapSaver]: map saved");

  return true;
}


//}

/* checkType() */ /*//{*/
template <typename OcTreeT>
bool OctomapSaver<OcTreeT>::checkType(std::string type_id) {
  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[OctomapSaver]: Cannot verify octomap type.");
  return true;  // Try deserialization, might crash though
}

template <>
bool OctomapSaver<octomap::OcTree>::checkType(std::string type_id) {
  return type_id == "OcTree";
}

template <>
bool OctomapSaver<octomap::ColorOcTree>::checkType(std::string type_id) {
  return type_id == "ColorOcTree";
}
/*//}*/

}  // namespace octomap_saver

}  // namespace mrs_octomap_tools

// Register as a component
typedef mrs_octomap_tools::octomap_saver::OctomapSaver<octomap::OcTree> OcTreeSaver;
typedef mrs_octomap_tools::octomap_saver::OctomapSaver<octomap::ColorOcTree> ColorOcTreeSaver;
RCLCPP_COMPONENTS_REGISTER_NODE(OcTreeSaver)
RCLCPP_COMPONENTS_REGISTER_NODE(ColorOcTreeSaver)
