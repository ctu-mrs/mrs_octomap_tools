#include <octomap_tools/octomap_methods.h>

namespace octomap_tools
{

/* setInsideBBX() //{ */

template <typename OcTree_t>
bool setInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const bool state) {

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

template <typename OcTree_t>
bool setFreeAboveGround(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max, const double height) {

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

      bool   got_ground = false;
      double ground_z   = 0;

      for (int k = 0; k < int((coord_max.z() - coord_min.z() + octree->getResolution()) / octree->getResolution()); k++) {

        double z = coord_max.z() - k * octree->getResolution();

        octomap::OcTreeKey   key  = octree->coordToKey(x, y, z);
        octomap::OcTreeNode* node = octree->search(key);

        if (node && octree->isNodeOccupied(node)) {
          got_ground = true;
          ground_z   = z;
          break;
        }
      }

      if (got_ground) {

        for (int k = 1; k < (height / octree->getResolution() + 1); k++) {

          double z = ground_z + k * octree->getResolution();

          octomap::OcTreeKey key = octree->coordToKey(x, y, z, octree->getTreeDepth());

          octree->setNodeValue(key, -1.0);
        }
      }
    }
  }

  return true;
}

//}

/* setUnknownInsideBBX() //{ */

template <typename OcTree_t>
bool setUnknownInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octree->deleteNode(it.getKey());
  }

  return true;
}

//}

/* pruneInBBX() //{ */

// | -------------------- DONT USE TO SLOW -------------------- |
template <typename OcTree_t>
bool pruneInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  mrs_lib::ScopeTimer scope_time("pruneInBBX");

  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(octree->getResolution());

  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMinLog());
  octree_new->setClampingThresMax(octree->getClampingThresMaxLog());

  copyInsideBBX2(octree, octree_new, p_min, p_max);

  octree_new->prune();

  setUnknownInsideBBX(octree, p_min, p_max);

  copyInsideBBX2(octree_new, octree, p_min, p_max);

  return true;
}

//}

/* expandInBBX() //{ */

template <typename OcTree_t>
bool expandInBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  return true;
}

//}

/* removeFreeInsideBBX() //{ */

// BUGGY
template <typename OcTree_t>
bool removeFreeInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = octree->search(k);

    expandNodeRecursive(octree, node, it.getDepth());
  }

  for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

    auto key  = it.getKey();
    auto node = octree->search(key);

    if (node && !octree->isNodeOccupied(node)) {
      octree->deleteNode(key);
    }
  }

  pruneInBBX(octree, p_min, p_max);

  return true;
}

//}

/* setUnknownToFreeInsideBBX() //{ */

template <typename OcTree_t>
bool setUnknownToFreeInsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

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

        octomap::OcTreeKey key  = octree->coordToKey(x, y, z);
        auto               node = octree->search(key);

        if (!node) {
          octree->setNodeValue(key, -1.0);
        }
      }
    }
  }

  return true;
}

//}

/* pruneNodeRecursive() //{ */

template <typename OcTree_t>
void pruneNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

  if (node_depth < octree->getTreeDepth()) {

    for (int i = 0; i < 8; i++) {

      auto child = octree->getNodeChild(node, i);

      pruneNodeRecursive(octree, child, node_depth + 1);
    }

    octree->pruneNode(node);

  } else {

    octree->pruneNode(node);

    return;
  }
}

//}

/* clearOutsideBBX() //{ */

template <typename OcTree_t>
bool clearOutsideBBX(std::shared_ptr<OcTree_t>& octree, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!octree->coordToKeyChecked(p_min, minKey) || !octree->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octree->expand();

  std::vector<std::pair<octomap::OcTreeKey, unsigned int>> keys;

  for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    // check if outside of bbx:
    octomap::OcTreeKey k = it.getKey();

    if (k[0] < minKey[0] || k[1] < minKey[1] || k[2] < minKey[2] || k[0] > maxKey[0] || k[1] > maxKey[1] || k[2] > maxKey[2]) {
      keys.push_back(std::make_pair(k, it.getDepth()));
    }
  }

  for (auto k : keys) {
    octree->deleteNode(k.first, k.second);
  }

  octree->prune();

  return true;
}

//}

/* setResolution() //{ */

template <typename OcTree_t>
bool setResolution(std::shared_ptr<OcTree_t>& octree, const double resolution) {

  octree->expand();

  double min_x, min_y, min_z;
  double max_x, max_y, max_z;

  octree->getMetricMin(min_x, min_y, min_z);
  octree->getMetricMax(max_x, max_y, max_z);

  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(resolution);

  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMinLog());
  octree_new->setClampingThresMax(octree->getClampingThresMaxLog());

  octomap::OcTreeKey min_key = octree_new->coordToKey(min_x, min_y, min_z);
  octomap::OcTreeKey max_key = octree_new->coordToKey(max_x, max_y, max_z);

  auto coord_min = octree_new->keyToCoord(min_key);
  auto coord_max = octree_new->keyToCoord(max_key);

  // changing to finer resolution
  if (resolution <= octree->getResolution()) {
    ROS_INFO("[OctomapEditor]: changing resolution to a finer one, this may take a while");
  }

  int x_steps = int((coord_max.x() - coord_min.x()) / resolution);
  int y_steps = int((coord_max.y() - coord_min.y()) / resolution);
  int z_steps = int((coord_max.z() - coord_min.z()) / resolution);

  // just for counting
  long points_total = x_steps * y_steps * z_steps;
  long point        = 0;

  for (int i = 0; i < x_steps; i++) {

    double x = coord_min.x() + i * resolution;

    for (int j = 0; j < y_steps; j++) {

      double y = coord_min.y() + j * resolution;

      for (int k = 0; k < z_steps; k++) {

        double z = coord_min.z() + k * resolution;

        point++;

        ROS_INFO_THROTTLE(0.1, "[OctomapEditor]: progress: %.2f%%", 100.0 * (double(point) / double(points_total)));

        // changing to finer resolution
        if (resolution <= octree->getResolution()) {

          octomap::OcTreeKey   key_old  = octree->coordToKey(x, y, z);
          octomap::OcTreeNode* node_old = octree->search(key_old);

          if (node_old) {
            octomap::OcTreeKey key_new = octree_new->coordToKey(x, y, z);

            octree_new->setNodeValue(key_new, node_old->getValue());
          }

          // changing to coarser resolution
        } else {

          octomap::point3d p_min(float(x - resolution / 2.0), float(y - resolution / 2.0), float(z - resolution / 2.0));
          octomap::point3d p_max(float(x + resolution / 2.0), float(y + resolution / 2.0), float(z + resolution / 2.0));

          float avg_value = 0;
          int   n_nodes   = 0;

          for (typename OcTree_t::leaf_bbx_iterator it = octree->begin_leafs_bbx(p_min, p_max), end = octree->end_leafs_bbx(); it != end; ++it) {

            octomap::OcTreeKey   k    = it.getKey();
            octomap::OcTreeNode* node = octree->search(k);

            if (node) {
              avg_value += (octree->isNodeOccupied(node)) ? 1.0 : -1.0;
              n_nodes++;
            }
          }

          if (n_nodes > 0) {

            avg_value /= float(n_nodes);

            octomap::OcTreeKey key_new = octree_new->coordToKey(x, y, z);

            octree_new->setNodeValue(key_new, avg_value);
          }
        }
      }
    }
  }

  octree_new->prune();

  octree = octree_new;

  ROS_INFO("[OctomapEditor]: resolution change finished");

  return true;
}

//}

/* refractor() //{ */

template <typename OcTree_t>
bool refractor(std::shared_ptr<OcTree_t>& octree, const int fractor, double& new_resolution) {

  mrs_lib::ScopeTimer timer("refractor");

  timer.checkpoint("pruning");

  octree->prune();

  timer.checkpoint("refractoring");

  double res_jump = pow(2.0, fractor);
  new_resolution  = octree->getResolution() * res_jump;

  std::shared_ptr<OcTree_t> octree_new;

  if (fractor < 0) {

    octree_new = std::make_shared<OcTree_t>(new_resolution);

    octree_new->setProbHit(octree->getProbHit());
    octree_new->setProbMiss(octree->getProbMiss());
    octree_new->setClampingThresMin(octree->getClampingThresMinLog());
    octree_new->setClampingThresMax(octree->getClampingThresMaxLog());
  }

  ROS_INFO("[OctomapEditor]: changing to resolution %.2f", new_resolution);

  /* octomap::OcTreeNode* root = octree_new->getRoot(); */

  /* bool got_root = root ? true : false; */

  int          cell_count = 0;
  unsigned int max_depth  = 0;

  ROS_INFO("[OctomapEditor]: orig tree depth %d", octree->getTreeDepth());

  if (fractor >= 0) {

    for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(octree->getTreeDepth() - fractor), end = octree->end_leafs(); it != end; ++it) {

      cell_count++;

      auto orig_key = it.getKey();

      const unsigned int old_depth = it.getDepth();

      /* octomap::OcTreeNode* orig_node = it.getNode(); */

      /* octree->eatChildren(orig_node); */

      if (old_depth >= max_depth) {
        max_depth = old_depth;
      }

      /* unsigned int new_depth; */
      /* if (old_depth <= (octree->getTreeDepth() - fractor)) { */
      /*   new_depth = old_depth; */
      /* } else { */
      /*   new_depth = octree->getTreeDepth() - fractor; */
      /* } */

      /* auto new_key_adj = octree->adjustKeyAtDepth(orig_key, new_depth); */
      /* octree_new->setNodeValue(new_key, orig_node->getValue()); */

      /* octomap::OcTreeKey new_key; */
      /* new_key.k[0] = ((orig_key.k[0] - 32768) / res_jump) + 32768; */
      /* new_key.k[1] = ((orig_key.k[1] - 32768) / res_jump) + 32768; */
      /* new_key.k[2] = ((orig_key.k[2] - 32768) / res_jump) + 32768; */

      /* int new_depth = old_depth + fractor; */

      /* auto new_key = octree_new->coordToKey(it.getX(), it.getY(), it.getZ()); */

      /* octomap::OcTreeNode* new_node = touchNode(octree_new, new_key, new_depth); */

      /* if (octree->isNodeOccupied(orig_node)) { */
      /*   new_node->setLogOdds(1.0); */
      /* } else { */
      /*   new_node->setLogOdds(-1.0); */
      /* } */
    }
  } else {

    for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

      cell_count++;

      unsigned int old_depth = it.getDepth();

      auto new_key = octree_new->coordToKey(it.getX(), it.getY(), it.getZ());

      auto node = touchNode(octree_new, new_key, old_depth + fractor);

      node->setValue(it->getValue());

      if (old_depth >= max_depth) {
        max_depth = old_depth;
      }
    }
  }

  ROS_INFO("[OctomapEditor]: cell count %d", cell_count);
  ROS_INFO("[OctomapEditor]: max depth %d", max_depth);

  if (fractor < 0) {
    octree = octree_new;
  }

  ROS_INFO("[OctomapEditor]: resolution change finished");

  return true;
}

//}

/* removeCeiling() //{ */

template <typename OcTree_t>
void removeCeiling(std::shared_ptr<OcTree_t>& octree, bool &is_expanded) {

  if (!is_expanded) {
    octree->expand();
    is_expanded = true;
  }

  const std::vector<int> below = {0, 0, -1};

  octomap::KeySet keys_to_remove;

  for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    if (!isCeilTopKey(octree, is_expanded, it.getKey())) {
      continue;
    }

    octomap::OcTreeKey tmp_key = it.getKey();

    int free_counter    = 0;
    int unknown_counter = 0;

    octomap::KeySet ceil_keys;
    ceil_keys.insert(tmp_key);

    // find the cells creating the whole ceiling

    while (true) {

      tmp_key = getKeyInDir(octree, tmp_key, below);

      octomap::OcTreeNode* tmp_node = octree->search(tmp_key);

      if (tmp_node) {

        if (!octree->isNodeOccupied(tmp_node)) {
          free_counter++;
        } else {
          free_counter    = 0;
          unknown_counter = 0;
          ceil_keys.insert(tmp_key);
        }

      } else {
        unknown_counter++;
      }

      if (free_counter + unknown_counter >= 5) {
        break;
      }
    }

    if (free_counter > unknown_counter) {
      for (auto& key : ceil_keys) {
        keys_to_remove.insert(key);
      }
    }
  }

  for (auto& key : keys_to_remove) {
    octree->deleteNode(key);
  }
}

//}

/* isCeilTopKey() //{ */

template <typename OcTree_t>
bool isCeilTopKey(std::shared_ptr<OcTree_t>& octree, bool& is_expanded, const octomap::OcTreeKey key) {

  if (!is_expanded) {
    octree->expand();
    is_expanded = true;
  }

  const std::vector<int> above = {0, 0, 1};

  octomap::OcTreeNode* node = octree->search(key);

  if (!octree->isNodeOccupied(node)) {
    return false;
  }

  octomap::OcTreeKey tmp_key = key;

  while (true) {

    tmp_key = getKeyInDir(octree, tmp_key, above);

    octomap::OcTreeNode* tmp_node = octree->search(tmp_key);

    if (!tmp_node) {
      return true;
    }

    if (octree->isNodeOccupied(tmp_node)) {
      return false;
    }
  }
}

//}

/* filterSpecs() //{ */

template <typename OcTree_t>
bool filterSpecs(std::shared_ptr<OcTree_t>& octree, bool &is_expanded, const int spec_size) {

  if (!is_expanded) {
    octree->expand();
  }

  octomap::KeySet keys_to_delete;
  octomap::KeySet touched_keys;

  for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    if (touched_keys.find(it.getKey()) != touched_keys.end()) {
      continue;
    }

    octomap::KeySet cluster;

    bool reached_end = getCluster(octree, it.getKey(), cluster, touched_keys, spec_size);

    if (reached_end) {
      for (auto& key : cluster) {
        keys_to_delete.insert(key);
      }
    }
  }

  for (auto& key : keys_to_delete) {
    octree->deleteNode(key);
  }

  return true;
}

//}

/* clear() //{ */

template <typename OcTree_t>
bool clear(std::shared_ptr<OcTree_t>& octree) {

  mrs_lib::ScopeTimer scope_timer("clear");

  octree->clear();

  return true;
}

//}

/* translateMap() //{ */

template <typename OcTree_t>
bool translateMap(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y, const double& z) {

  ROS_INFO("[OctomapServer]: translating map by %.2f, %.2f, %.2f", x, y, z);

  octree->expand();

  // allocate the new future octree
  std::shared_ptr<OcTree_t> octree_new = std::make_shared<OcTree_t>(octree->getResolution());
  octree_new->setProbHit(octree->getProbHit());
  octree_new->setProbMiss(octree->getProbMiss());
  octree_new->setClampingThresMin(octree->getClampingThresMin());
  octree_new->setClampingThresMax(octree->getClampingThresMax());

  for (typename OcTree_t::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {

    auto coords = it.getCoordinate();

    coords.x() += float(x);
    coords.y() += float(y);
    coords.z() += float(z);

    auto value = it->getValue();
    auto key   = it.getKey();

    auto new_key = octree_new->coordToKey(coords);

    octree_new->setNodeValue(new_key, value);
  }

  octree_new->prune();

  octree = octree_new;

  ROS_INFO("[OctomapServer]: map translated");

  return true;
}

//}

/* copyInsideBBX() //{ */

// SLOW AND DEPRICATED
template <typename OcTree_t>
bool copyInsideBBX(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  mrs_lib::ScopeTimer scope_time("copyInsideBBX()");

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  for (typename OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = from->search(k);

    expandNodeRecursive(from, node, it.getDepth());
  }

  for (typename OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    to->setNodeValue(it.getKey(), it->getValue());
  }

  return true;
}

//}

/* copyInsideBBX2() //{ */

template <typename OcTree_t>
bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  mrs_lib::ScopeTimer scope_time("copyInsideBBX2()");

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->setNodeValue(key, 1.0);
  }

  for (typename OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(p_min.x() - to->getResolution() * 2.0, p_min.y(), p_min.z(), to->getTreeDepth());
    to->deleteNode(key, to->getTreeDepth());
  }

  return true;
}

//}

/* touchNode() //{ */

template <typename OcTree_t>
octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth) {

  octomap::OcTreeNode* root = octree->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {

    octomap::OcTreeKey temp_key = octree->coordToKey(0, 0, 0);

    octree->updateNode(temp_key, false);
  }

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

template <typename OcTree_t>
octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                     unsigned int max_depth) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {
    return node;
  }
}

//}

/* morphologyOperation() //{ */

template <typename OcTree_t>
bool morphologyOperation(std::shared_ptr<OcTree_t>& octree, Morphology_t operation, const octomap::point3d& p_min, const octomap::point3d& p_max) {

  expandInBBX(octree, p_min, p_max);

  /* double min_x, min_y, min_z; */
  /* double max_x, max_y, max_z; */

  /* octree_->getMetricMin(min_x, min_y, min_z); */
  /* octree_->getMetricMax(max_x, max_y, max_z); */

  /* octomap::point3d p_min(min_x, min_y, min_z); */
  /* octomap::point3d p_max(max_x, max_y, max_z); */

  /* p_min -= octomap::point3d(5, 5, 5); */
  /* p_max += octomap::point3d(5, 5, 5); */

  std::shared_ptr<OcTree_t> octree_filtered = std::make_shared<OcTree_t>(octree->getResolution());

  octree_filtered->setProbHit(octree->getProbHit());
  octree_filtered->setProbMiss(octree->getProbMiss());
  octree_filtered->setClampingThresMin(octree->getClampingThresMinLog());
  octree_filtered->setClampingThresMax(octree->getClampingThresMaxLog());

  octomap::OcTreeKey min_key, max_key;

  if (!octree->coordToKeyChecked(p_min, min_key) || !octree->coordToKeyChecked(p_max, max_key)) {
    return false;
  }

  auto coord_min = octree->keyToCoord(min_key);
  auto coord_max = octree->keyToCoord(max_key);

  int x_steps = int((coord_max.x() - coord_min.x()) / octree->getResolution());
  int y_steps = int((coord_max.y() - coord_min.y()) / octree->getResolution());
  int z_steps = int((coord_max.z() - coord_min.z()) / octree->getResolution());

  long points_total = x_steps * y_steps * z_steps;
  long point        = 0;

  for (int i = 0; i < x_steps; i++) {

    double x = coord_min.x() + i * octree->getResolution();

    for (int j = 0; j < y_steps; j++) {

      double y = coord_min.y() + j * octree->getResolution();

      for (int k = 0; k < z_steps; k++) {

        point++;

        ROS_INFO_THROTTLE(0.1, "[OctomapEditor]: progress: %.2f%%", 100.0 * (double(point) / double(points_total)));

        double z = coord_min.z() + k * octree->getResolution();

        octomap::OcTreeKey   key  = octree->coordToKey(x, y, z, octree->getTreeDepth());
        octomap::OcTreeNode* node = octree->search(key);

        std::vector<octomap::OcTreeKey> neighbours = getNeighborhood(octree, key);

        int count_free     = 0;
        int count_occupied = 0;
        int count_unknown  = 0;

        for (auto& n : neighbours) {

          octomap::OcTreeNode* nnode = octree->search(n);

          if (nnode && octree->isNodeOccupied(nnode)) {
            count_occupied++;
          } else if (nnode && !octree->isNodeOccupied(nnode)) {
            count_free++;
          } else {
            count_unknown++;
          }
        }

        if (operation == DILATE) {
          if (count_occupied >= 1) {
            octree_filtered->setNodeValue(key, 1.0);
          } else {
            if (node) {
              octree_filtered->setNodeValue(key, node->getValue());
            }
          }
        } else if (operation == ERODE) {
          if (node && octree->isNodeOccupied(node) && (count_occupied < 26)) {
            octree_filtered->setNodeValue(key, -1.0);
          } else {
            if (node) {
              octree_filtered->setNodeValue(key, node->getValue());
            }
          }
        }
      }
    }
  }

  octree_filtered->prune();

  copyInsideBBX(octree_filtered, octree, p_min, p_max);

  return true;
}

//}

/* getClusters() //{ */

template <typename OcTree_t>
bool getCluster(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, const int max_size) {

  auto node = octree->search(key);

  if (node && octree->isNodeOccupied(node)) {

    int cluster_size = 0;

    return getClusterRecurs(octree, key, visited, touched, cluster_size, max_size);

  } else {

    return true;
  }
}

//}

/* getClustersRecurs() //{ */

template <typename OcTree_t>
bool getClusterRecurs(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, octomap::KeySet& visited, octomap::KeySet& touched, int& cluster_size,
                      const int max_size) {

  visited.insert(key);
  touched.insert(key);

  cluster_size++;

  if (cluster_size < max_size) {

    for (auto& d : EXPANSION_DIRECTIONS) {

      auto newkey = getKeyInDir(octree, key, d);

      if (visited.find(newkey) != visited.end()) {
        continue;
      }

      if (touched.find(newkey) != touched.end()) {
        return false;
      }

      auto node = octree->search(newkey);

      if (node && octree->isNodeOccupied(node)) {

        bool reached_end = getClusterRecurs(octree, newkey, visited, touched, cluster_size, max_size);

        if (!reached_end) {
          return false;
        }
      }
    }

  } else {

    for (auto& d : EXPANSION_DIRECTIONS) {

      auto newkey = getKeyInDir(octree, key, d);

      if (visited.find(newkey) != visited.end()) {
        continue;
      }

      if (touched.find(newkey) != touched.end()) {
        return false;
      }

      auto node = octree->search(newkey);

      if (node && octree->isNodeOccupied(node)) {

        touched.insert(newkey);
        return false;
      }
    }
  }

  return true;
}

//}

/* getNeighborKeys() //{ */

template <typename OcTree_t>
std::vector<octomap::OcTreeKey> getNeighborhood(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key) {

  std::vector<octomap::OcTreeKey> neighbors;

  for (auto& d : EXPANSION_DIRECTIONS) {

    auto newkey = getKeyInDir(octree, key, d);

    neighbors.push_back(newkey);
  }

  return neighbors;
}

//}

/* getKeyInDir() //{ */

template <typename OcTree_t>
octomap::OcTreeKey getKeyInDir(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, const std::vector<int>& direction) {

  octomap::OcTreeKey k;

  k.k[0] = key.k[0] + direction[0];
  k.k[1] = key.k[1] + direction[1];
  k.k[2] = key.k[2] + direction[2];

  return k;
}

//}

/* expandNodeRecursive() //{ */

template <typename OcTree_t>
void expandNodeRecursive(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const unsigned int node_depth) {

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

/* checkType() *//*//{*/
template <typename OcTreeT>
bool checkType(std::string type_id)
{
  //General case: Need to be specialized for every used case
  /* setStatus(StatusProperty::Warn, "Messages", QString("Cannot verify octomap type")); */
  ROS_WARN_THROTTLE(2.0, "[Octomap_saver]: Cannot verify octomap type.");
  return true; //Try deserialization, might crash though
}
  
/* template <> */
/* bool checkType<octomap::OcTreeStamped>(std::string type_id) */
/* { */
/*   if(type_id == "OcTreeStamped") return true; */
/*   else return false; */
/* } */

template <>
bool checkType<octomap::OcTree>(std::string type_id)
{
  if(type_id == "OcTree") return true;
  else return false;
}

template <>
bool checkType<octomap::ColorOcTree>(std::string type_id)
{
  if(type_id == "ColorOcTree") return true;
  else return false;
}
/*//}*/

}  // namespace octomap_tools
