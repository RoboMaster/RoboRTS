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

#ifndef ROBORTS_CAMERA_CAMERA_NODE_H
#define ROBORTS_CAMERA_CAMERA_NODE_H

#include <thread>
#include <vector>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <actionlib/server/simple_action_server.h>
#include <image_transport/image_transport.h>

#include "uvc/uvc_driver.h"

#include "camera_param.h"
#include "camera_base.h"
#include "alg_factory/algorithm_factory.h"
#include "io/io.h"

namespace roborts_camera{

class CameraNode{
 public:
  /**
   * @brief Construcor of camera node
   * @details Read the camera parameter, create the camera drivers based on camera factory
   * and start all camera threads for update images.
   */
  explicit CameraNode();
  /**
   * @brief Start all the threads for camera drivers to update and publish the image data
   */
  void StartThread();
  /**
   * @brief Invoke each camera driver to read the image data and publish using ROS image_transport with corresponding parameter
   */
  void Update(const unsigned int camera_num_);
  /**
 * @brief Stop to read image.
 */
  void StoptThread();
  ~CameraNode();
 private:
  //! drivers of different cameras inherited from camera base
  std::vector<std::shared_ptr<CameraBase>> camera_driver_;
  //! camera parameters
  CameraParam camera_param_;
  //! number of cameras
  unsigned long camera_num_;
  //! flag of running
  bool running_;
  //! threads of different cameras
  std::vector<std::thread> camera_threads_;

  //! ROS node handlers of different cameras
  std::vector<ros::NodeHandle> nhs_;
  //! ROS image transport camera publisher to publish image data
  std::vector<image_transport::CameraPublisher> img_pubs_;

};
} //namespace roborts_camera

#endif //ROBORTS_CAMERA_CAMERA_NODE_H
