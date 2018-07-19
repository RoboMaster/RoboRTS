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

#ifndef AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
#define AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H

//system include
#include <vector>
#include <list>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/opencv.hpp"

#include "modules/perception/detection/armor_detection/constraint_set/constraint_set.h"
#include "modules/perception/detection/armor_detection/armor_detection_base.h"
#include "modules/perception/detection/armor_detection/constraint_set/proto/constraint_set.pb.h"
#include "modules/perception/detection/util/cv_toolbox.h"

#include "common/algorithm_factory.h"
#include "common/error_code.h"

namespace rrts {
namespace perception {
namespace detection {

using rrts::common::ErrorCode;
using rrts::common::ErrorInfo;

enum State {
  INITIALIZED = 0,
  RUNNING = 1,
  FAILED = 2,
  STOPED = 3
};

/**
 *  This class describes the armor information, including maximum bounding box, vertex, standard deviation.
 */
class ArmorInfo {
 public:
  ArmorInfo(cv::RotatedRect armor_rect, std::vector<cv::Point2f> armor_vertex, float armor_stddev = 0.0) {
    rect = armor_rect;
    vertex = armor_vertex;
    stddev = armor_stddev;
  }
 public:
  cv::RotatedRect rect;
  std::vector<cv::Point2f> vertex;
  float stddev;
};

/**
 * @brief This class achieved functions that can help to detect armors of RoboMaster vehicle.
 */
class ConstraintSet : public ArmorDetectionBase {
 public:
  ConstraintSet();
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam() override;
  /**
   * @brief The entrance function of armor detection.
   * @param translation Translation information of the armor relative to the camera.
   * @param rotation Rotation information of the armor relative to the camera.
   */
  ErrorInfo DetectArmor(bool &detected, double &x, double &y, double &z, double &distance, double &pitch, double &yaw) override;
  /**
   * @brief Detecting lights on the armors.
   * @param src Input image
   * @param lights Output lights information
   */
  void DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights);
  /**
   * @brief Filtering the detected lights.
   * @param lights Filtered lights
   */
  void FilterLights(std::vector<cv::RotatedRect> &lights);
  /**
   * @brief Finding possible armors.
   * @param lights Take lights information as input.
   * @param armors Possible armors
   */
  void PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors);
  /**
   * @brief Filtering Detected armors by standard deviation and non-maximum suppression(nms).
   * @param armors Result armors
   */
  void FilterArmors(std::vector<ArmorInfo> &armors);
  /**
   * @brief Slecting final armor as the target armor which we will be shot.
   * @param Input armors
   */
  ArmorInfo SlectFinalArmor(std::vector<ArmorInfo> &armors);
  /**
   *
   * @param armor
   * @param distance
   * @param pitch
   * @param yaw
   * @param bullet_speed
   */
  void CalcControlInfo(const ArmorInfo & armor,
                       double &x,
                       double &y,
                       double &z,
                       double &distance,
                       double &pitch,
                       double &yaw,
                       double bullet_speed);
  /**
   * @brief Using two lights(left light and right light) to calculate four points of armor.
   * @param armor_points Out put
   * @param left_light Rotated rect of left light
   * @param right_light Rotated rectangles of right light
   */
  void CalcArmorInfo(std::vector<cv::Point2f> &armor_points, cv::RotatedRect left_light, cv::RotatedRect right_light);
  /**
   * @brief Calculating the coordinates of the armor by its width and height.
   * @param width Armor width
   * @param height Armor height
   */
  void SolveArmorCoordinate(const float width, const float height);
  /**
   *
   */
  void SignalFilter(double &new_num, double &old_num,unsigned int &filter_count, double max_diff);
  /**
   * @brief Destructor
   */
  ~ConstraintSet() final;
 private:
  ErrorCode error_code_;
  ErrorInfo error_info_;
  unsigned int filter_x_count_;
  unsigned int filter_y_count_;
  unsigned int filter_z_count_;
  unsigned int filter_distance_count_;
  unsigned int filter_pitch_count_;
  unsigned int filter_yaw_count_;

  rrts::driver::camera::CameraParam cameras_;
  int camera_id_;
  cv::VideoCapture cap_;
  CVToolbox cv_toolbox_;

  cv::Mat src_img_;
  cv::Mat gray_img_;
  // Parameters come form .prototxt fileen
  bool enable_debug_;
  bool enable_neon_;
  bool using_hsv_;
  unsigned int enemy_color_;
  float gimbal_offset_z_;
  float camera_and_armor_diff_;
  float optical_axis_offset_;
  float yaw_offset_;
  float pitch_offset_;

  cv::Mat show_lights_before_filter_;
  cv::Mat show_lights_after_filter_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;

  //armor info
  std::vector<cv::Point3f> armor_points_;
  std::list<double> yaws_in_moment_;

  //Filter lights
  float light_max_aspect_ratio_;
  float light_min_area_;
  float light_max_angle_;
  float light_max_angle_diff_;

  //Filter armor
  float armor_max_angle_;
  float armor_min_area_;
  float armor_max_aspect_ratio_;
  float armor_max_pixel_val_;
  float armor_max_mean_;
  float armor_max_stddev_;

  float color_thread_;
  float blue_thread_;
  float red_thread_;

  unsigned int max_wait_fps_;
  float min_pulse_angle_;
  unsigned int min_num_;

  double old_x_;
  double old_y_;
  double old_z_;

  double old_distance_;
  double old_pitch_;
  double old_yaw_;
  
  bool old_updated_;

  //ros
  ros::NodeHandle nh;
  std::vector<image_transport::Publisher> subscribers_;
  cv::VideoCapture cap_handle_;
};

rrts::common::REGISTER_ALGORITHM(ArmorDetectionBase, "constraint_set", ConstraintSet);

} //namespace detection
} //namespace perception
} //namespace rrts

#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
