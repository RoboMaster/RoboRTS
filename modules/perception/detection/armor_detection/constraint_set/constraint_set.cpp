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
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>

#include "modules/perception/detection/armor_detection/constraint_set/constraint_set.h"

#include "common/timer.h"
#include "common/io.h"
#include "common/log.h"

namespace rrts{
namespace perception {
namespace detection {

ConstraintSet::ConstraintSet() {
  LoadParam();
  image_transport::ImageTransport it(nh);
  error_info_ = ErrorInfo(rrts::common::OK);
}

void ConstraintSet::LoadParam() {
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name =
      "modules/perception/detection/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  CHECK(read_state) << "Cannot open " << file_name;

  camera_id_ = constraint_set_config_.init_camera_id();
  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();

  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
}

ErrorInfo ConstraintSet::DetectArmor(double &distance, double &pitch, double &yaw) {
  TIMER_START(DetectArmor)
  std::vector<cv::RotatedRect> lights;
  std::vector<ArmorInfo> armors;

  cv_toolbox_.NextImage(src_img_, camera_id_);
  if (!src_img_.empty()) {
    NOTICE("Begin to detect armor!")
    cv::cvtColor(src_img_, gray_img_, CV_BGR2GRAY);
    if (enable_debug_) {
      show_lights_before_filter_ = src_img_.clone();
      show_lights_after_filter_ = src_img_.clone();
      show_armors_befor_filter_ = src_img_.clone();
      show_armors_after_filter_ = src_img_.clone();
      cv::waitKey(1);
    }

    DetectLights(src_img_, lights);
    FilterLights(lights);
    PossibleArmors(lights, armors);
    FilterArmors(armors);

    if (!armors.empty()) {
      ArmorInfo final_armor = SlectFinalArmor(armors);
      CalcControlInfo(final_armor, distance, pitch, yaw, 10);
    }
    lights.clear();
    armors.clear();
  } else {
    NOTICE("Waiting for run camera driver...")
  }
//  if (enable_debug_)
//    TIMER_END(DetectArmor)
  return error_info_;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
  auto light = cv_toolbox_.DistillationColor(src, enemy_color_);
  cv::Mat binary_brightness_img;
  cv::Mat binary_color_img;
  cv::Mat binary_light_img;

  //TODO(noah.guo): param
  cv::threshold(gray_img_, binary_brightness_img, 200, 255, CV_THRESH_BINARY);
  //TODO(noah.guo): param
  float thresh;
  if (enemy_color_ == BLUE)
    thresh = 90;
  else
    thresh = 50;
  cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(binary_color_img, binary_color_img, element, cv::Point(-1, -1), 1);
  binary_light_img = binary_color_img & binary_brightness_img;
  if (enable_debug_)
    cv::imshow("binary_light_img", binary_light_img);

  auto contours_light = cv_toolbox_.FindContours(binary_light_img);
  auto contours_brightness = cv_toolbox_.FindContours(binary_brightness_img);

  lights.reserve(contours_brightness.size());
  // TODO: To be optimized
  std::vector<int> is_processes(contours_brightness.size());
  for (unsigned int i = 0; i < contours_light.size(); ++i) {
    for (unsigned int j = 0; j < contours_brightness.size(); ++j) {
      if (!is_processes[j]) {
        if (cv::pointPolygonTest(contours_brightness[j], contours_light[i][0], true) >= 0.0) {
          cv::RotatedRect single_light = cv::minAreaRect(contours_brightness[j]);
          lights.push_back(single_light);
          if (enable_debug_)
            cv_toolbox_.DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(100), 2);
          is_processes[j] = true;
          break;
        }
      }
    }
  }
}

void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) {
  std::vector<cv::RotatedRect> rects;
  rects.reserve(lights.size());

  for (const auto &light : lights) {
    float angle = 0.0f;
    auto light_aspect_ratio =
        std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
    angle = light.angle >= 90.0 ? std::abs(light.angle - 90.0) : std::abs(light.angle);

    if (light_aspect_ratio < light_max_aspect_ratio_ ||
        angle < light_max_angle_ && light.size.area() >= light_min_area_) {
      rects.push_back(light);
      if (enable_debug_)
        cv_toolbox_.DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(100), 2);
    }
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);

  lights = rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) {
  for (const auto &light1 : lights) {
    for (const auto &light2 : lights) {
      auto edge1 = std::minmax(light1.size.width, light1.size.height);
      auto edge2 = std::minmax(light2.size.width, light2.size.height);
      auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
          (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
      auto center_angle =
          std::atan((light1.center.y - light2.center.y) / (light1.center.x - light2.center.x)) * 180 / CV_PI;

      cv::RotatedRect rect;
      rect.angle = static_cast<float>(center_angle);
      rect.center.x = (light1.center.x + light2.center.x) / 2;
      rect.center.y = (light1.center.y + light2.center.y) / 2;
      float armor_width = static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first);
      float armor_height = std::max<float>(edge1.second, edge2.second);

      rect.size.width = std::max<float>(armor_width, armor_height);
      rect.size.height = std::min<float>(armor_width, armor_height);

      if (std::abs(light1.angle - light2.angle) < light_max_angle_diff_ &&

          std::abs(center_angle) < armor_max_angle_ &&
          rect.size.width / (float) (rect.size.height) < armor_max_aspect_ratio_ &&
          rect.size.area() > armor_min_area_ &&
          gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
              < armor_max_pixel_val_) {

        if (light1.center.x < light2.center.x) {
          std::vector<cv::Point2f> armor_points;
          CalArmorInfo(armor_points, light1, light2);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_.DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(100), 2);
          armor_points.clear();
        } else {
          std::vector<cv::Point2f> armor_points;
          CalArmorInfo(armor_points, light2, light1);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_.DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(100), 2);
          armor_points.clear();
        }
      }
    }
  }
  if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  cv::Mat mask = cv::Mat::zeros(gray_img_.size(), CV_8UC1);
  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    cv::Point pts[4];
    for (unsigned int i = 0; i < 4; i++) {
      pts[i].x = (int) armor_iter->vertex[i].x;
      pts[i].y = (int) armor_iter->vertex[i].y;
    }
    cv::fillConvexPoly(mask, pts, 4, cv::Scalar(255), 8, 0);

    cv::Mat mat_mean;
    cv::Mat mat_stddev;
    cv::meanStdDev(gray_img_, mat_mean, mat_stddev, mask);

    auto stddev = mat_stddev.at<double>(0, 0);

    if (stddev > armor_max_stddev_) {
      armor_iter = armors.erase(armor_iter);
    } else {
      armor_iter++;
    }
  }

  // nms
  std::vector<bool> is_armor(armors.size(), true);
  for (int i = 0; i < armors.size() && is_armor[i] == true; i++) {
    for (int j = i + 1; j < armors.size() && is_armor[j]; j++) {
      float dx = armors[i].rect.center.x - armors[j].rect.center.x;
      float dy = armors[i].rect.center.y - armors[j].rect.center.y;
      float dis = std::sqrt(dx * dx + dy * dy);
      if (dis < armors[i].rect.size.width + armors[j].rect.size.width) {
        if (armors[i].rect.angle > armors[j].rect.angle)
          is_armor[i] = false;
        else
          is_armor[j] = false;
      }
    }
  }

  for (auto armor_iter = armors.begin(); armor_iter != armors.end();) {
    if (!is_armor[armor_iter - armors.begin()])
      armor_iter = armors.erase(armor_iter);
    else if (enable_debug_) {
      cv_toolbox_.DrawRotatedRect(show_armors_after_filter_, armor_iter->rect, cv::Scalar(255, 255, 255), 2);
      armor_iter++;
    } else
      armor_iter++;
  }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);
}

ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
  std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

  if (enable_debug_) {
    cv_toolbox_.DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(100), 2);
    cv::imshow("relust_img_", src_img_);
  }
  return armors[0];
}

void ConstraintSet::CalcControlInfo(const ArmorInfo & armor,
                                    double &distance,
                                    double &pitch,
                                    double &yaw,
                                    double bullet_speed) {
  cv::Mat rvec;
  cv::Mat tvec;
  cv::solvePnP(armor_points_,
               armor.vertex,
               cameras_.GetCameraParam()[camera_id_].camera_matrix,
               cameras_.GetCameraParam()[camera_id_].camera_distortion,
               rvec,
               tvec);

  double fly_time = tvec.at<double>(2) / 1000.0 / bullet_speed;
  double gravity_offset = 0.5 * 9.8 * fly_time * fly_time * 1000;
  const double gimble_offset = 3.3;
  double xyz[3] = {tvec.at<double>(0), tvec.at<double>(1) - gravity_offset + gimble_offset, tvec.at<double>(2)};

  //calculate pitch
  pitch = atan(-xyz[1]/xyz[2]);
  //calculate yaw
  yaw = atan2(xyz[0], xyz[2]);

  //radian to angle
  pitch = pitch * 180 / M_PI;
  yaw   = yaw * 180 / M_PI;

  distance = sqrt(tvec.at<double>(0)*tvec.at<double>(0) + tvec.at<double>(2)*tvec.at<double>(2));
}

void ConstraintSet::CalArmorInfo(std::vector<cv::Point2f> &armor_points,
                                 cv::RotatedRect left_light,
                                 cv::RotatedRect right_light) {
  cv::Point2f left_points[4], right_points[4];
  left_light.points(left_points);
  right_light.points(right_points);
  cv::Point2f right_lu, right_ld, lift_ru, lift_rd;
  std::sort(left_points, left_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  std::sort(right_points, right_points + 4, [](const cv::Point2f &p1, const cv::Point2f &p2) { return p1.x < p2.x; });
  if (right_points[0].y < right_points[1].y) {
    right_lu = right_points[0];
    right_ld = right_points[1];
  } else {
    right_lu = right_points[1];
    right_ld = right_points[0];
  }

  if (left_points[2].y < left_points[3].y) {
    lift_ru = left_points[2];
    lift_rd = left_points[3];
  } else {
    lift_ru = left_points[3];
    lift_rd = left_points[2];
  }
  armor_points.push_back(lift_ru);
  armor_points.push_back(right_lu);
  armor_points.push_back(right_ld);
  armor_points.push_back(lift_rd);
}

void ConstraintSet::SolveArmorCoordinate(const float width,
                                         const float height) {
  armor_points_.emplace_back(cv::Point3f(-width/2, height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  height/2,  0.0));
  armor_points_.emplace_back(cv::Point3f(width/2,  -height/2, 0.0));
  armor_points_.emplace_back(cv::Point3f(-width/2, -height/2, 0.0));
}

ConstraintSet::~ConstraintSet() {
  cv_toolbox_.StopReadCamera();
  cap_.release();
}
} //namespace detection
} //namespace perception
} //namespace rrts