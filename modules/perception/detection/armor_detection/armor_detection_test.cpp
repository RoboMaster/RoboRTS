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

#include <ros/ros.h>
#include "boost/thread.hpp"

#include "modules/perception/detection/armor_detection/armor_detection_base.h"
#include "modules/perception/detection/armor_detection/proto/armor_detection.pb.h"
#include "modules/perception/detection/armor_detection/armor_detection_algorithms.h"

#include "common/algorithm_factory.h"
#include "common/io.h"
#include "common/log.h"

using namespace rrts::perception::detection;

int main(int argc, char **argv) {
  ros::init(argc, argv, "test");
  ArmorDetectionAlgorithms armor_detection_algorithms;
  bool read_state = rrts::common::ReadProtoFromTextFile("/modules/perception/detection/armor_detection/config/armor_detection.prototxt",
                                                        &armor_detection_algorithms);
  if (!read_state) {
    LOG_ERROR << "Cannot open .prototxt file!";
  }
  std::string selected_algorithm = armor_detection_algorithms.selected_algorithm();

  //create the selected algorithms
  std::unique_ptr<ArmorDetectionBase> armor_detector = \
      rrts::common::AlgorithmFactory<ArmorDetectionBase>::CreateAlgorithm(selected_algorithm);
  while(1){
    std::vector<float> translation;
    std::vector<float> rotation;
    armor_detector->DetectArmor(translation, rotation);
    //TODO(noah.guo): coordinat transformation
  }
  return 0;
}