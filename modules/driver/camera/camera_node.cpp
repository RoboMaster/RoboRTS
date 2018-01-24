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

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "modules/driver/camera/camera_node.h"
#include "common/main_interface.h"

namespace rrts {
namespace driver {
namespace camera {
CameraNode::CameraNode(std::string name) : rrts::common::RRTS::RRTS(name) {
  LoadParam();
  publishers_.resize(cameras_.size());
  image_transport::ImageTransport it(nh);
  for (unsigned int i = 0; i < cameras_.size(); i++) {
    std::string topic_name = "camera_" + std::to_string(cameras_[i].camera_id);
    publishers_[i] = it.advertise(topic_name, 2);
  }
  running_ = true;
  StartReadCamera();
}
void CameraNode::LoadParam() {
  camera_param_.GetCameraParam(cameras_);
}
/**
 * @brief
 */
void CameraNode::StartReadCamera() {
  for (auto &iter: cameras_) {
    unsigned int camera_id = iter.camera_id;
    if (iter.mode == 0)
      iter.cap_handle.open(camera_id);
    else
      iter.cap_handle.open(iter.video_path);
    CHECK(iter.cap_handle.isOpened()) << "Cannot open camera " << camera_id << ".";
    unsigned int index = &iter - &cameras_[0];
    iter.camera_thread = new std::thread(&CameraNode::Update, this, index);
  }
}
/**
 *
 */
void CameraNode::Update(const unsigned int &index) {
  while (running_) {
    cv::Mat frame;
    cameras_[index].cap_handle >> frame;
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
    publishers_[index].publish(msg);
  }
  std::cout << "2edfrewf" << std::endl;
}
/**
 * @brief Stop to read image.
 */
void CameraNode::StopReadCamera() {
}
CameraNode::~CameraNode() {
  running_ = false;
  for (auto &iter: cameras_) {
    if (iter.camera_thread->joinable())
      iter.camera_thread->join();
  }
}
} //namespace camera
} //namespace drivers
} //namespace rrts

MAIN(rrts::driver::camera::CameraNode, "camera_node")
