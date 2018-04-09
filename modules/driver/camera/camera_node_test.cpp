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

#include "modules/driver/camera/camera_node.h"
#include "common/main_interface.h"

namespace rrts {
namespace driver {
namespace camera {
CameraNode::CameraNode(std::string name): rrts::common::RRTS::RRTS(name) {
  camera_num_ = camera_param_.GetCameraParam().size();
  publishers_.resize(camera_num_);
  camera_threads_.resize(camera_num_);
  camera_driver_.resize(camera_num_);
  image_transport::ImageTransport it(nh);
  for (unsigned int i = 0; i < camera_num_; i++) {
    std::string topic_name = "camera_" + std::to_string(i);
    publishers_[i] = it.advertise(topic_name, 2);
    //create the selected camera driver
    std::string camera_type = camera_param_.GetCameraParam()[i].camera_type;
    camera_driver_[i] = rrts::common::AlgorithmFactory<CameraBase>::CreateAlgorithm(camera_type);
  }
  running_ = true;
  StartThread();
}
void CameraNode::StartThread() {
  for (unsigned int i = 0; i < camera_num_; i++) {
    camera_threads_[i] = std::thread(&CameraNode::Update, this, i);
  }
}
/**
 *
 */
void CameraNode::Update(const unsigned int index) {
  cv::Mat img;
  while(running_) {
    camera_driver_[index]->StartReadCamera(index, img);
    if(!img.empty()) {
      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
      publishers_[index].publish(msg);
    }
  }
}
/**
 * @brief Stop to read image.
 */
void CameraNode::StoptThread() {

}

CameraNode::~CameraNode() {
  running_ = false;
  for (auto &iter: camera_threads_) {
    if (iter.joinable())
      iter.join();
  }
}
} //namespace camera
} //namespace drivers
} //namespace rrts

int main(int argc, char **argv){
  rrts::common::GLogWrapper glog_wrapper(argv[0]);
  signal(SIGINT, SignalHandler);
  signal(SIGTERM,SignalHandler);
  ros::init(argc, argv, "test", ros::init_options::NoSigintHandler);
  rrts::driver::camera::CameraNode rrts("test");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
