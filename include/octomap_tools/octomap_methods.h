#ifndef OCTOMAP_METHODS_H
#define OCTOMAP_METHODS_H

#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <mrs_lib/scope_timer.h>
#include <ros/ros.h>

/* using OcTree_t = octomap::OcTree; */

namespace octomap_tools
{

const std::vector<std::vector<int>> EXPANSION_DIRECTIONS = {{-1, -1, -1}, {-1, -1, 0}, {-1, -1, 1}, {-1, 0, -1}, {-1, 0, 0}, {-1, 0, 1}, {-1, 1, -1},
                                                            {-1, 1, 0},   {-1, 1, 1},  {0, -1, -1}, {0, -1, 0},  {0, -1, 1}, {0, 0, -1}, {0, 0, 1},
                                                            {0, 1, -1},   {0, 1, 0},   {0, 1, 1},   {1, -1, -1}, {1, -1, 0}, {1, -1, 1}, {1, 0, -1},
                                                            {1, 0, 0},    {1, 0, 1},   {1, 1, -1},  {1, 1, 0},   {1, 1, 1}};

typedef enum
{
  DILATE,
  ERODE,
} Morphology_t;



template <typename OcTree_t>
void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);


template <typename OcTree_t>
void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

template <typename OcTree_t>
bool getCluster(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, const int max_size);

template <typename OcTree_t>
bool getClusterRecurs(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, int& cluster_size,
                      const int max_size);

template <typename OcTree_t>
octomap::OcTreeKey getKeyInDir(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, const std::vector<int>& direction);

template <typename OcTree_t>
std::vector<octomap::OcTreeKey> getNeighborhood(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key);

template <typename OcTree_t>
void pruneNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

template <typename OcTree_t>
bool setInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max, const bool state);

template <typename OcTree_t>
bool setUnknownInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool pruneInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool expandInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool removeFreeInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool setUnknownToFreeInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool setFreeAboveGround(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const double height);

template <typename OcTree_t>
bool clearOutsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool setResolution(std::shared_ptr<OcTree_t>& octree, const double resolution);

template <typename OcTree_t>
bool refractor(std::shared_ptr<OcTree_t>& octree, const int fractor, double& new_resolution);

template <typename OcTree_t>
bool filterSpecs(std::shared_ptr<OcTree_t>& octree, bool& is_expanded, const int spec_size);

template <typename OcTree_t>
void removeCeiling(std::shared_ptr<OcTree_t>& octree, bool& is_expanded);

template <typename OcTree_t>
bool isCeilTopKey(std::shared_ptr<OcTree_t>& octree, bool& is_expanded, const octomap::OcTreeKey key);

template <typename OcTree_t>
bool copy(std::shared_ptr<OcTree_t>& octree);

template <typename OcTree_t>
bool clear(std::shared_ptr<OcTree_t>& octree);

template <typename OcTree_t>
bool copyInsideBBX(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

template <typename OcTree_t>
bool morphologyOperation(std::shared_ptr<OcTree_t>& octree, Morphology_t operation, const octomap::point3d& p_min, const octomap::point3d& p_max);

template <typename OcTree_t>
octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                     unsigned int max_depth = 0);

template <typename OcTree_t>
octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0);

///Returns false, if the type_id (of the message) does not correspond to the template paramter
///of this class, true if correct or unknown (i.e., no specialized method for that template).
template <typename OcTree_t>
bool checkType(std::string type_id);

}  // namespace octomap_tools

#endif  // OCTOMAP_METHODS_H
