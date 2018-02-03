/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
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

#ifndef MODULES_STREAM_TF_TREE_TF_TREE_H
#define MODULES_STREAM_TF_TREE_TF_TREE_H

#include <string>
#include <thread>

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/tf.h"

#include "modules/stream/tf_tree/proto/tf_tree.pb.h"

#include "common/io.h"
#include "common/rrts.h"
#include "common/log.h"

namespace rrts {
namespace stream {
namespace tf_tree {

class tf_node {
 public:
  tf_node(tf::Transform transform, std::string base, std::string sensor){
    transform_ = transform;
    base_      = base;
    sensor_    = sensor;
  }
  ~tf_node(){}
 public:
  tf::Transform transform_;
  std::string base_;
  std::string sensor_;
};

class TFTree: public rrts::common::RRTS {
public:
  explicit TFTree(std::string name);
  void Execute();
  template<typename T>
  void AddLeaf(const T *coordinate_trans);

  void PubStaticTF(unsigned int tf_num);
  ~TFTree() final;
 private:
  bool running_;
  int tf_num_;
  CoordinateTrans *coordinate_trans_;
  std::thread *tf_tree_thread_;

  //ros
  tf::TransformBroadcaster tf_broadcaster_;
  tf::Transform transform_;
  std::vector<tf_node> tf_nodes_;
};

} //namespace tf_tree
} //namespace stream
} //namespace rrts
#endif //MODULES_STREAM_TF_TREE_TF_TREE_H