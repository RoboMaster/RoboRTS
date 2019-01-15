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

#include <queue>
#include "particle_filter_kdtree.h"

namespace roborts_localization{

ParticleFilterKDTree::~ParticleFilterKDTree() {
	delete (root_ptr_);
	while (nodes_ptr_vec_.empty()) {
		delete (nodes_ptr_vec_.back());
		nodes_ptr_vec_.pop_back();
	}
	nodes_ptr_vec_.clear();
	nodes_ptr_vec_.shrink_to_fit();
	DLOG_INFO << "Destroy kd tree";
}

void ParticleFilterKDTree::InitializeByMaxSize(int max_size) {

	size_[0] = 0.5;
	size_[1] = 0.5;
	size_[2] = (10.0 * M_PI / 180.0);
	node_count_ = 0;
	node_max_count_ = max_size;
	nodes_ptr_vec_.resize(node_max_count_);
	leaf_count_ = 0;
}

const int &ParticleFilterKDTree::GetLeafCount() const {
	return leaf_count_;
}

void ParticleFilterKDTree::Clear() {
	root_ptr_ = nullptr;
	while (nodes_ptr_vec_.empty()) {
		delete (nodes_ptr_vec_.back());
		nodes_ptr_vec_.pop_back();
	}
	nodes_ptr_vec_.clear();
	nodes_ptr_vec_.shrink_to_fit();
	leaf_count_ = 0;
	node_count_ = 0;
}

void ParticleFilterKDTree::InsertPose(Vec3d pose, double value) {
	Vec3d key;
	for (int i = 0; i < 3; i++) {
		key[i] = std::floor(pose[i] / size_[i]);
	}
	root_ptr_ = InsertNode(nullptr, root_ptr_, key, value);
}

ParticleFilterKDTreeNode *ParticleFilterKDTree::InsertNode(
		ParticleFilterKDTreeNode *parent_node_ptr,
		ParticleFilterKDTreeNode *node_ptr,
		const Vec3d &key,
		double value) {

	int i = 0;
	int split = 0, max_split = 0;
	if (node_ptr == nullptr) {
		assert(node_count_ < node_max_count_);
		node_count_++;
		node_ptr = (new ParticleFilterKDTreeNode);
		nodes_ptr_vec_.push_back(node_ptr);
		nodes_ptr_vec_.back()->leaf = true;
		if (parent_node_ptr == nullptr) {
			nodes_ptr_vec_.back()->depth = 0;
		} else {
			nodes_ptr_vec_.back()->depth = parent_node_ptr->depth + 1;
		}
		nodes_ptr_vec_.back()->key = key;
		nodes_ptr_vec_.back()->value = value;
		this->leaf_count_ += 1;
		//return nodes_ptr_vec_.back();

	} else if (node_ptr->leaf) {
		if (IsEqualKey(key, node_ptr->key)) {
			node_ptr->value += value;
		} else {
			max_split = 0;
			node_ptr->pivot_dim = -1;
			for (i = 0; i < 3; i++) {
				split = static_cast<int>(std::fabs(key[i] - node_ptr->key[i]));
				if (split > max_split) {
					max_split = split;
					node_ptr->pivot_dim = i;
				}
			}
			assert(node_ptr->pivot_dim >= 0);

			node_ptr->pivot_value = (key(node_ptr->pivot_dim) + node_ptr->key(node_ptr->pivot_dim)) / 2.0;

			if (key[node_ptr->pivot_dim] < node_ptr->pivot_value) {
				node_ptr->left_ptr = InsertNode(node_ptr, nullptr, key, value);
				node_ptr->right_ptr = InsertNode(node_ptr, nullptr, node_ptr->key, node_ptr->value);
			} else {
				node_ptr->left_ptr = InsertNode(node_ptr, nullptr, node_ptr->key, node_ptr->value);
				node_ptr->right_ptr = InsertNode(node_ptr, nullptr, key, value);
			}

			node_ptr->leaf = false;
			this->leaf_count_ -= 1;
		}
	} else {
		assert(node_ptr->left_ptr != nullptr);
		assert(node_ptr->right_ptr != nullptr);

		if (key[node_ptr->pivot_dim] < node_ptr->pivot_value)
			InsertNode(node_ptr, node_ptr->left_ptr, key, value);
		else
			InsertNode(node_ptr, node_ptr->right_ptr, key, value);

	}
	return node_ptr;
}

int ParticleFilterKDTree::GetCluster(Vec3d pose) {
	Vec3d key;
	ParticleFilterKDTreeNode *node_ptr;

	for (int i = 0; i < pose.size(); i++) {
		key(i) = std::floor(pose(i) / size_[i]);
	}

	node_ptr = FindNode(root_ptr_, key);
	if (node_ptr == nullptr) {
		LOG_WARNING << "Cluster not found";
		return -1;
	}
	return node_ptr->cluster;

}

ParticleFilterKDTreeNode *ParticleFilterKDTree::FindNode(ParticleFilterKDTreeNode *node_ptr, Vec3d key) {
	if (node_ptr->leaf) {
		if (IsEqualKey(key, node_ptr->key)) {
			return node_ptr;
		} else {
			return nullptr;
		}
	} else {
		assert(node_ptr->left_ptr != nullptr);
		assert(node_ptr->right_ptr != nullptr);
		// If the keys are different...
		if (key[node_ptr->pivot_dim] < node_ptr->pivot_value)
			return FindNode(node_ptr->left_ptr, key);
		else
			return FindNode(node_ptr->right_ptr, key);
	}

}

inline bool ParticleFilterKDTree::IsEqualKey(const Vec3d &key_a, const Vec3d &key_b) {
	return key_a == key_b;
}

void ParticleFilterKDTree::Cluster() {

	int i = 0;
	int queue_count = 0, cluster_count = 0;
	ParticleFilterKDTreeNode *node_ptr;

	std::unique_ptr<ParticleFilterKDTreeNode * []> queue(new ParticleFilterKDTreeNode *[this->node_count_]);

	for (i = 0; i < this->node_count_; i++) {
		node_ptr = this->nodes_ptr_vec_[i];
		if (node_ptr->leaf) {
			node_ptr->cluster = -1;
			assert(queue_count < this->node_count_);
			queue[queue_count++] = node_ptr;
		}
	}

	cluster_count = 0;

	while (queue_count > 0) {
		node_ptr = queue[--queue_count];

		if (node_ptr->cluster >= 0) {
			continue;
		}

		node_ptr->cluster = cluster_count++;
		ClusterNode(node_ptr, 0);
	}
	queue.reset();
//	delete (queue);
	DLOG_INFO << "Cluster count = " << cluster_count;
}

void ParticleFilterKDTree::ClusterNode(ParticleFilterKDTreeNode *node_ptr, int depth) {
	int i;
	Vec3d nkey;
	ParticleFilterKDTreeNode *n_node_ptr;

	for (i = 0; i < 3 * 3 * 3; i++) {
		nkey(0) = node_ptr->key(0) + (i / 9) - 1;
		nkey(1) = node_ptr->key(1) + ((i % 9) / 3) - 1;
		nkey(2) = node_ptr->key(2) + ((i % 9) % 3) - 1;

		n_node_ptr = FindNode(root_ptr_, nkey);
		if (n_node_ptr == nullptr) {
			continue;
		}

		assert(n_node_ptr->leaf);

		// This node already has a label; skip it.  The label should be
		// consistent, however.
		if (n_node_ptr->cluster >= 0) {
			continue;
		}

		n_node_ptr->cluster = node_ptr->cluster;
		ClusterNode(n_node_ptr, depth + 1);
	}
	return;
}

}// roborts_localization