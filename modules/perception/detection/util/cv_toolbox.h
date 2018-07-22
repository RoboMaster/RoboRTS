/****************************************************************************
 *  Copyright (C) 2018 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify *  it under the terms of the GNU General Public License as published by *  the Free Software Foundation, either version 3 of the License, or *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef MODULES_DETECTION_CVTOOLBOX_H
#define MODULES_DETECTION_CVTOOLBOX_H

#include <vector>
#include <thread>
#include <mutex>
//opencv
#include "opencv2/opencv.hpp"
//ros
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "modules/perception/detection/armor_detection/constraint_set/proto/constraint_set.pb.h"
#include "modules/driver/camera/camera_param.h"
#include "common/log.h"

namespace rrts {
namespace perception {
namespace detection {

/**
 * This class is a toolbox for RoboMaster detection.
 */
class CVToolbox {
 public:
  /**
   *  @brief The constructor of the CVToolbox.
   */
  CVToolbox(unsigned int buffer_size = 2,
            unsigned int camera_index = 0) :
      buffer_size_(buffer_size),
      camera_index_(camera_index) {

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    unsigned long camera_num = camera_param_.GetCameraParam().size();
    subs_.resize(camera_num);
    image_buffer_.resize(camera_num);
    write_index_.resize(buffer_size);
    read_index_.resize(buffer_size);
    for (unsigned long i = 0; i < camera_num; i++) {
      image_buffer_[i].resize(buffer_size);
      write_index_[i] = 0;
      read_index_[i] = 1;
    }

    for (unsigned int i = 0; i < subs_.size(); i++) {
      std::string topic_name = "camera_" + std::to_string(i);
      subs_[i] = it.subscribe(topic_name, 20, boost::bind(&CVToolbox::ReceiveImg, this, _1, i));
    }
  }

  void ReceiveImg(const sensor_msgs::ImageConstPtr &msg, unsigned int camera_id) {
    image_buffer_[camera_id][write_index_[camera_id]] = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    lock_.lock();
    unsigned int tmp_index_ = write_index_[camera_id];
    write_index_[camera_id] = read_index_[camera_id];
    read_index_[camera_id] = tmp_index_;
    lock_.unlock();
  }
  /**
   * @brief Get next new image.
   * @param src_img Output image
   */
  void NextImage(cv::Mat &src_img, int camera_id) {
    lock_.lock();
    image_buffer_.at(camera_id).at(read_index_.at(camera_id)).copyTo(src_img);
    lock_.unlock();
  }
  /**
   *  @brief Stop to read image.
   */
  void StopReadCamera() {
  }
  /**
   * @brief Highlight the blue or red region of the image.
   * @param image Input image ref
   * @return Single channel image
   */
  cv::Mat DistillationColor(const cv::Mat &src_img, unsigned int color, bool using_hsv) {
    if(using_hsv) {
      cv::Mat img_hsv;
      cv::cvtColor(src_img, img_hsv, CV_BGR2HSV);
      if (color == BLUE) {
        cv::Mat img_hsv_blue, img_threshold_blue;
        img_hsv_blue = img_hsv.clone();
        cv::Mat blue_low(cv::Scalar(90, 150, 46));
        cv::Mat blue_higher(cv::Scalar(140, 255, 255));
        cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
        return img_threshold_blue;
      } else {
        cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
        img_hsv_red1 = img_hsv.clone();
        img_hsv_red2 = img_hsv.clone();
        cv::Mat red1_low(cv::Scalar(0, 43, 46));
        cv::Mat red1_higher(cv::Scalar(3, 255, 255));

        cv::Mat red2_low(cv::Scalar(170, 43, 46));
        cv::Mat red2_higher(cv::Scalar(180, 255, 255));
        cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
        cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
        img_threshold_red = img_threshold_red1 | img_threshold_red2;
        //cv::imshow("img_threshold_red", img_threshold_red);
        return img_threshold_red;
      }
    } else {
      std::vector<cv::Mat> bgr;
      cv::split(src_img, bgr);
      if (color == RED) {
        cv::Mat result_img;
        cv::subtract(bgr[2], bgr[1], result_img);
        return result_img;
      } else if (color == BLUE) {
        cv::Mat result_img;
        cv::subtract(bgr[0], bgr[2], result_img);
        return result_img;
      }
    }
  }
  /**
   * @brief The wrapper for function cv::findContours
   * @param binary binary image ref
   * @return Contours that found.
   */
  std::vector<std::vector<cv::Point>> FindContours(const cv::Mat &binary_img) {
    std::vector<std::vector<cv::Point>> contours;
    const auto mode = CV_RETR_EXTERNAL;
    const auto method = CV_CHAIN_APPROX_SIMPLE;
    cv::findContours(binary_img, contours, mode, method);
    return contours;
  }
  /**
   * @brief Draw rectangle.
   * @param img The image will be drew on
   * @param rect The target rectangle
   * @param color Rectangle color
   * @param thickness Thickness of the line
   */
  void DrawRotatedRect(const cv::Mat &img, const cv::RotatedRect &rect, const cv::Scalar &color, int thickness) {
    cv::Point2f vertex[4];

    rect.points(vertex);
    for (int i = 0; i < 4; i++)
      cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
  }
 private:
  unsigned int buffer_size_;

  std::vector<std::vector<cv::Mat>> image_buffer_;
  rrts::driver::camera::CameraParam camera_param_;
  unsigned int camera_index_;
  std::vector<unsigned int> write_index_;
  std::vector<unsigned int> read_index_;
  std::mutex lock_;

  std::vector<image_transport::Subscriber> subs_;
};
} //namespace detection
} //namespace perception
} //namespace rrts

#endif //MODULES_DETECTION_CVTOOLBOX_H
