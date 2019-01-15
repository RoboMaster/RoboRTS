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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include <vector>
#include <thread>
#include <mutex>
//opencv
#include "opencv2/opencv.hpp"
//ros
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

std::string topic_name = "back_camera";
cv::VideoWriter writer(topic_name+".avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, cv::Size(640, 360));
cv::Mat src_img;

void ReceiveImg(const sensor_msgs::ImageConstPtr &msg) {
  src_img = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  writer.write(src_img);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "image_capture");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber sub = it.subscribe(topic_name, 20, boost::bind(&ReceiveImg, _1));

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::waitForShutdown();
  return 0;
}
