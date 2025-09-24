/* includes //{ */

#include <rclcpp/rclcpp.hpp>
#include <octomap/OcTree.h>
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

class OctomapCeilingRemover : public rclcpp::Node {

public:
  explicit OctomapCeilingRemover(const rclcpp::NodeOptions & options);

private:

  bool is_initialized_ = false;

  std::shared_ptr<mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>> sh_octomap_;

  void callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg);

  // | ------------------------ routines ------------------------ |

  bool saveToFile(std::shared_ptr<octomap::OcTree>& octree, const std::string& filename);
};

//}

/* constructor //{ */

OctomapCeilingRemover::OctomapCeilingRemover(const rclcpp::NodeOptions & options)
  : Node("octomap_ceiling_remover", options)
{
  RCLCPP_INFO(this->get_logger(), "[OctomapCeilingRemover]: initializing");

  mrs_lib::ParamLoader param_loader(this->shared_from_this(), "OctomapCeilingRemover");

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(this->get_logger(), "[OctomapCeilingRemover]: could not load all parameters");
    rclcpp::shutdown();
    return;
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;
  shopts.node               = this->shared_from_this();
  shopts.node_name          = "OctomapCeilingRemover";
  shopts.no_message_timeout = mrs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.queue_size         = 1;

  auto callback = [this](const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {
      this->callbackOctomap(msg);
    };

  sh_octomap_ = std::make_shared<mrs_lib::SubscriberHandler<octomap_msgs::msg::Octomap>>(shopts, "octomap_in", callback);

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(this->get_logger(), "[OctomapCeilingRemover]: initialized");
}
//}

/* callbackOctomap() //{ */

void OctomapCeilingRemover::callbackOctomap(const octomap_msgs::msg::Octomap::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[OctomapCeilingRemover]: getting octomap");

  octomap_msgs::msg::Octomap::ConstSharedPtr octomap = msg;

  octomap::AbstractOcTree* tree_ptr;

  if (octomap->binary) {
    tree_ptr = octomap_msgs::binaryMsgToMap(*octomap);
  } else {
    tree_ptr = octomap_msgs::fullMsgToMap(*octomap);
  }

  if (!tree_ptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[OctomapCeilingRemover]: octomap message is empty!");
    return;
  }

  std::shared_ptr<octomap::OcTree> octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(tree_ptr));

  octree->expand();
}

//}

//}

}  // namespace octomap_ceiling_remover

}  // namespace mrs_octomap_tools

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(mrs_octomap_tools::octomap_ceiling_remover::OctomapCeilingRemover)