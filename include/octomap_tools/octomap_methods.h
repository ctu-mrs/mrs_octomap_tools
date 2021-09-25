#ifndef OCTOMAP_METHODS_H
#define OCTOMAP_METHODS_H

#include <octomap/OcTree.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>
#include <octomap/AbstractOccupancyOcTree.h>
#include <mrs_lib/scope_timer.h>
#include <ros/ros.h>

using OcTree_t = octomap::OcTree;

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


void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

bool getCluster(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, const int max_size);

bool getClusterRecurs(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, int& cluster_size,
                      const int max_size);

octomap::OcTreeKey getKeyInDir(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, const std::vector<int>& direction);

std::vector<octomap::OcTreeKey> getNeighborhood(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key);

void pruneNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth);

bool setInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max, const bool state);

bool setUnknownInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool pruneInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool expandInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool removeFreeInsideBBX(std::shared_ptr<OcTree_t>& from, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool setUnknownToFreeInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool setFreeAboveGround(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const double height);

bool clearOutsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool setResolution(std::shared_ptr<OcTree_t>& octree, const double resolution);

bool refractor(std::shared_ptr<OcTree_t>& octree, const int fractor, double& new_resolution);

bool filterSpecs(std::shared_ptr<OcTree_t>& octree, bool& is_expanded, const int spec_size);

void removeCeiling(std::shared_ptr<OcTree_t>& octree, bool& is_expanded);

bool isCeilTopKey(std::shared_ptr<OcTree_t>& octree, bool& is_expanded, const octomap::OcTreeKey key);

bool copy(std::shared_ptr<OcTree_t>& octree);

bool clear(std::shared_ptr<OcTree_t>& octree);

bool copyInsideBBX(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z);

bool morphologyOperation(std::shared_ptr<OcTree_t>& octree, Morphology_t operation, const octomap::point3d& p_min, const octomap::point3d& p_max);

octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                     unsigned int max_depth);

octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);


}  // namespace octomap_tools

#endif  // OCTOMAP_METHODS_H
