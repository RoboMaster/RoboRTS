/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PATICLE_FILTER_KDTREE_H
#define ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PATICLE_FILTER_KDTREE_H

#include <cmath>
#include <memory>

#include "log.h"
#include "localization_math.h"

namespace roborts_localization{

/**
 * @brief Information for a particle filter K-D tree node
 */
class ParticleFilterKDTreeNode {
 public:
  //! Left child node
  ParticleFilterKDTreeNode *left_ptr;
  //! Right child node
  ParticleFilterKDTreeNode *right_ptr;
  //! Leaf node flag
  bool leaf = false;
  //! Depth in the tree
  int depth = 0;
  //! The cluster label
  int cluster = -1;
  //! Pivot dimension
  int pivot_dim = 0;
  //! Pivot value
  double pivot_value = 0;
  //! The key for this node
  Vec3d key = {0, 0, 0};
  //! The value for this node
  double value = 0;
};

/**
 * @brief K-D tree class
 */
class ParticleFilterKDTree {
 public:
  /**
   * @brief Create a K-D tree by max sample size
   * @param max_size Max sample size
   */
  void InitializeByMaxSize(int max_size);
  /**
   * @brief Clear all entries from the tree
   */
  void Clear();
  /**
   * @brief Insert a pose into the tree
   * @param pose Pose to insert
   * @param value Weight of pose
   */
  void InsertPose(Vec3d pose, double value);
  /**
   * @brief Cluster the leaves in the tree
   */
  void Cluster();
  /**
   * @brief Determine the cluster label for the given pose
   * @param pose Pose
   * @return Returns the cluster label of given pose if exist.
   */
  int GetCluster(Vec3d pose);
  /**
   * @brief Get number of leaves
   * @return Returns number of leaves.
   */
  const int &GetLeafCount() const;
  /**
   * @brief Destuctor
   */
  virtual ~ParticleFilterKDTree();
 private:
  ParticleFilterKDTreeNode *InsertNode(ParticleFilterKDTreeNode *parent_node_ptr,
                                       ParticleFilterKDTreeNode *node_ptr,
                                       const Vec3d &key,
                                       double value);
  ParticleFilterKDTreeNode *FindNode(ParticleFilterKDTreeNode *node_ptr, Vec3d key);
  inline bool IsEqualKey(const Vec3d &key_a, const Vec3d &key_b);
  void ClusterNode(ParticleFilterKDTreeNode *node_ptr, int depth);
 private:
  //! Cell size
  double size_[3] = {0.5,0.5,(10.0 * M_PI / 180.0)};
  //! The root node of the tree
  ParticleFilterKDTreeNode *root_ptr_ = nullptr;
  //! The number of nodes in the tree
  int node_count_ = 0;
  //! Max number of nodes in the tree
  int node_max_count_ = 0;
  //! Vector of nodes
  std::vector<ParticleFilterKDTreeNode *> nodes_ptr_vec_;
  //! The number of leaf nodes in the tree
  int leaf_count_ = 0;
};

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_AMCL_PARTICLE_FILTER_PATICLE_FILTER_KDTREE_H
