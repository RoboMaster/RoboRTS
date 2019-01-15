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

#ifndef ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
#define ROBORTS_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H

//system include
#include <vector>
#include <list>

#include <opencv2/opencv.hpp>

#include "alg_factory/algorithm_factory.h"
#include "state/error_code.h"

#include "cv_toolbox.h"

#include "../armor_detection_base.h"

#include "proto/constraint_set.pb.h"
#include "constraint_set.h"
namespace roborts_detection {

using roborts_common::ErrorCode;
using roborts_common::ErrorInfo;

enum State {
  INITIALIZED = 0,
  RUNNING = 1,
  FAILED = 2,
  STOPED = 3
};

struct LightInfo {

  explicit LightInfo(cv::Point2f vertices[]) {
    auto edge_1 = std::pow(vertices[0].x - vertices[1].x, 2) +
        std::pow(vertices[0].y - vertices[1].y, 2);
    auto edge_2 = std::pow(vertices[1].x - vertices[2].x, 2) +
        std::pow(vertices[1].y - vertices[2].y, 2);

    if (edge_1 > edge_2) {
      width_  = (float)std::sqrt(edge_1);
      height_ = (float)std::sqrt(edge_2);

      if (vertices[0].y < vertices[1].y) {
        angle_ = std::atan2(vertices[1].y - vertices[0].y, vertices[1].x - vertices[0].x);
      } else {
        angle_ = std::atan2(vertices[0].y - vertices[1].y, vertices[0].x - vertices[1].x);
      }

    } else {
      width_  = (float)std::sqrt(edge_2);
      height_ = (float)std::sqrt(edge_1);

      if (vertices[2].y < vertices[1].y) {
        angle_ = std::atan2(vertices[1].y - vertices[2].y, vertices[1].x - vertices[2].x);
      } else {
        angle_ = std::atan2(vertices[2].y - vertices[1].y, vertices[2].x - vertices[1].x);
      }

    }

    angle_ = (float)(angle_*180.0 / 3.1415926);
    area_ = width_ * height_;
    aspect_ratio_ = width_ / height_;
    center_.x = (vertices[1].x + vertices[3].x) / 2;
    center_.y = (vertices[1].y + vertices[3].y) / 2;
    vertices_.push_back(vertices[0]);
    vertices_.push_back(vertices[1]);
    vertices_.push_back(vertices[2]);
    vertices_.push_back(vertices[3]);
  }

 public:
  //! Light area
  float area_;
  //! Light angle, come from the long edge's slope
  float angle_;
  //! Light center
  cv::Point2f center_;
  //! Light aspect ratio = width_/height_
  float aspect_ratio_;
  //! Light width
  float width_;
  //! Light height
  float height_;
  //! Light vertices
  std::vector<cv::Point2f> vertices_;
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
  ConstraintSet(std::shared_ptr<CVToolbox> cv_toolbox);
  /**
   * @brief Loading parameters from .prototxt file.
   */
  void LoadParam() override;
  /**
   * @brief The entrance function of armor detection.
   * @param translation Translation information of the armor relative to the camera.
   * @param rotation Rotation information of the armor relative to the camera.
   */
  ErrorInfo DetectArmor(bool &detected, cv::Point3f &target_3d) override;
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
  void CalcControlInfo(const ArmorInfo & armor, cv::Point3f &target_3d);

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

  void SetThreadState(bool thread_state) override;
  /**
   * @brief Destructor
   */
  ~ConstraintSet() final;
 private:
  ErrorInfo error_info_;
  unsigned int filter_x_count_;
  unsigned int filter_y_count_;
  unsigned int filter_z_count_;
  unsigned int filter_distance_count_;
  unsigned int filter_pitch_count_;
  unsigned int filter_yaw_count_;

  cv::Mat src_img_;
  cv::Mat gray_img_;
  //!  Camera intrinsic matrix
  cv::Mat intrinsic_matrix_;
  //! Camera distortion Coefficient
  cv::Mat distortion_coeffs_;
  //! Read image index
  int read_index_;
  //! detection time
  double detection_time_;

  // Parameters come form .prototxt file
  bool enable_debug_;
  bool using_hsv_;
  unsigned int enemy_color_;

  //! Use for debug
  cv::Mat show_lights_before_filter_;
  cv::Mat show_lights_after_filter_;
  cv::Mat show_armors_befor_filter_;
  cv::Mat show_armors_after_filter_;

  //! armor info
  std::vector<cv::Point3f> armor_points_;

  //! Filter lights
  std::vector<LightInfo> lights_info_;
  float light_max_aspect_ratio_;
  float light_min_area_;
  float light_max_angle_;
  float light_max_angle_diff_;

  //! Filter armor
  float armor_max_angle_;
  float armor_min_area_;
  float armor_max_aspect_ratio_;
  float armor_max_pixel_val_;
  float armor_max_mean_;
  float armor_max_stddev_;

  float color_thread_;
  float blue_thread_;
  float red_thread_;

  bool thread_running_;

  //ros
  ros::NodeHandle nh;
};

roborts_common::REGISTER_ALGORITHM(ArmorDetectionBase, "constraint_set", ConstraintSet, std::shared_ptr<CVToolbox>);

} //namespace roborts_detection

#endif // AOTO_PILOT_DETECTION_ARMOR_DETECTION_CONSTRAINT_SET_H
