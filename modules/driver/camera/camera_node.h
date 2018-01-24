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

#ifndef DRIVERS_CAMERA_NODE_H
#define DRIVERS_CAMERA_NODE_H

#include <thread>

#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "actionlib/server/simple_action_server.h"

#include "modules/driver/camera/camera_param.h"
#include "common/rrts.h"
#include "common/io.h"
#include "common/log.h"
namespace rrts {
namespace driver {
namespace camera {

class CameraNode : public rrts::common::RRTS {
 public:
  CameraNode(std::string name);
  void LoadParam();
  /**
   * Initializing the image buffer and creating a new thread to read camera.
   */
  void StartReadCamera();
  void Update(const unsigned int &index);
  void StopReadCamera();
  ~CameraNode() override;
 private:
  CameraParam camera_param_;
  std::vector<CameraInfo> cameras_;
  bool read_camera_initialized_;
  bool running_;

  //ros
  ros::NodeHandle nh;
  std::vector<image_transport::Publisher> publishers_;
  //actionlib::SimpleActionServer<messages::exampleAction> as_;
};
} //namespace camera
} //namespace driver
} //namespace rrts
#endif //DRIVERS_CAMERA_NODE_H
