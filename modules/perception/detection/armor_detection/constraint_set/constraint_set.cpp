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
  filter_x_count_ = 0;
  filter_y_count_ = 0;
  filter_z_count_ = 0;
  filter_distance_count_ = 0;
  filter_pitch_count_ = 0;
  filter_yaw_count_ = 0;
  old_updated_ = false;
  //cap_handle_.open("/home/yanan.guo/data/armor_detection/2.avi");

  LoadParam();
  image_transport::ImageTransport it(nh);
  error_info_ = ErrorInfo(rrts::common::OK);
}

void ConstraintSet::LoadParam() {
  //read parameters
  ConstraintSetConfig constraint_set_config_;
  std::string file_name =
      "/modules/perception/detection/armor_detection/constraint_set/config/constraint_set.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &constraint_set_config_);
  CHECK(read_state) << "Cannot open " << file_name;

  camera_id_ = constraint_set_config_.init_camera_id();
  enable_debug_ = constraint_set_config_.enable_debug();
  enemy_color_ = constraint_set_config_.enemy_color();
  using_hsv_ = constraint_set_config_.using_hsv();
  gimbal_offset_z_ = constraint_set_config_.gimbal_offset_z();
  camera_and_armor_diff_ = constraint_set_config_.camera_and_armor_diff();
  optical_axis_offset_ = constraint_set_config_.optical_axis_offset();
  yaw_offset_ = constraint_set_config_.yaw_offset();
  pitch_offset_ = constraint_set_config_.pitch_offset();
  //armor info
  float armor_width = constraint_set_config_.armor_size().width();
  float armor_height = constraint_set_config_.armor_size().height();
  SolveArmorCoordinate(armor_width, armor_height);

  //algorithm threshold parameters
  light_max_aspect_ratio_ = constraint_set_config_.threshold().light_max_aspect_ratio();
  light_min_area_ = constraint_set_config_.threshold().light_min_area();
  light_max_angle_ = constraint_set_config_.threshold().light_max_angle();
  light_max_angle_diff_ = constraint_set_config_.threshold().light_max_angle_diff();
  armor_max_angle_ = constraint_set_config_.threshold().armor_max_angle();
  armor_min_area_ = constraint_set_config_.threshold().armor_min_area();
  armor_max_aspect_ratio_ = constraint_set_config_.threshold().armor_max_aspect_ratio();
  armor_max_pixel_val_ = constraint_set_config_.threshold().armor_max_pixel_val();
  armor_max_stddev_ = constraint_set_config_.threshold().armor_max_stddev();
  armor_max_mean_   = constraint_set_config_.threshold().armor_max_mean();

  color_thread_ = constraint_set_config_.threshold().color_thread();
  blue_thread_ = constraint_set_config_.threshold().blue_thread();
  red_thread_ = constraint_set_config_.threshold().red_thread();

  max_wait_fps_ = constraint_set_config_.signal_recognization().max_wait_fps();
  min_pulse_angle_ = constraint_set_config_.signal_recognization().min_pulse_angle();
  min_num_ = constraint_set_config_.signal_recognization().min_num();
}

ErrorInfo ConstraintSet::DetectArmor(bool &detected, double &x, double &y, double &z, double &distance, double &pitch, double &yaw) {
  TIMER_START(DetectArmor)
  std::vector<cv::RotatedRect> lights;
  std::vector<ArmorInfo> armors;

  //cap_handle_ >> src_img_;
  //int c = cv::waitKey(10);
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

    if(!armors.empty()) {
      detected = true;
      ArmorInfo final_armor = SlectFinalArmor(armors);
      cv_toolbox_.DrawRotatedRect(src_img_, armors[0].rect, cv::Scalar(0, 255, 0), 2);
      CalcControlInfo(final_armor, x, y, z, distance, pitch, yaw, 10);
    } else
      detected = false;
    if(enable_debug_) {
      cv::imshow("relust_img_", src_img_);
    }
//    if(armors.empty()) {
//      cvWaitKey(0);
//    }

    lights.clear();
    armors.clear();
  } else {
    NOTICE("Waiting for run camera driver...")
    usleep(1000);
  }
  //if (enable_debug_)
  //TIMER_END(DetectArmor)
  return error_info_;
}

void ConstraintSet::DetectLights(const cv::Mat &src, std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************DetectLights********************************************" << std::endl;
  cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::dilate(src, src, element, cv::Point(-1, -1), 1);
  cv::Mat binary_brightness_img, binary_light_img, binary_color_img;
  if(using_hsv_) {
    binary_color_img = cv_toolbox_.DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
  }else {
    auto light = cv_toolbox_.DistillationColor(src, enemy_color_, using_hsv_);
    cv::threshold(gray_img_, binary_brightness_img, color_thread_, 255, CV_THRESH_BINARY);
    float thresh;
    if (enemy_color_ == BLUE)
      thresh = blue_thread_;
    else
      thresh = red_thread_;
    cv::threshold(light, binary_color_img, thresh, 255, CV_THRESH_BINARY);
    if(enable_debug_)
      cv::imshow("light", light);
  }
  binary_light_img = binary_color_img & binary_brightness_img;
  if (enable_debug_) {
    cv::imshow("binary_brightness_img", binary_brightness_img);
    cv::imshow("binary_light_img", binary_light_img);
    cv::imshow("binary_color_img", binary_color_img);
  }

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
            cv_toolbox_.DrawRotatedRect(show_lights_before_filter_, single_light, cv::Scalar(0, 255, 0), 2);
          is_processes[j] = true;
          break;
        }
      }
    }
  }
  if (enable_debug_)
    cv::imshow("show_lights_before_filter", show_lights_before_filter_);
}


void ConstraintSet::FilterLights(std::vector<cv::RotatedRect> &lights) {
  //std::cout << "********************************************FilterLights********************************************" << std::endl;
  std::vector<cv::RotatedRect> rects;
  rects.reserve(lights.size());

  for (const auto &light : lights) {
    float angle = 0.0f;
    auto light_aspect_ratio =
        std::max(light.size.width, light.size.height) / std::min(light.size.width, light.size.height);
    //https://stackoverflow.com/questions/15956124/minarearect-angles-unsure-about-the-angle-returned/21427814#21427814
    if(light.size.width < light.size.height) {
      angle = - light.angle;
    } else
      angle = light.angle + 90;
    //std::cout << "light angle: " << angle << std::endl;
    //std::cout << "light_aspect_ratio: " << light_aspect_ratio << std::endl;
    //std::cout << "light_area: " << light.size.area() << std::endl;
    if (light_aspect_ratio < light_max_aspect_ratio_ &&
        angle < light_max_angle_                     && 
        light.size.area() >= light_min_area_) {

      rects.push_back(light);
      if (enable_debug_)
        cv_toolbox_.DrawRotatedRect(show_lights_after_filter_, light, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_)
    cv::imshow("lights_after_filter", show_lights_after_filter_);

  lights = rects;
}

void ConstraintSet::PossibleArmors(const std::vector<cv::RotatedRect> &lights, std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************PossibleArmors********************************************" << std::endl;
  for (unsigned int i = 0; i < lights.size(); i++) {
    for (unsigned int j = i; j < lights.size(); j++) {
      cv::RotatedRect light1 = lights[i];
      cv::RotatedRect light2 = lights[j];
      auto edge1 = std::minmax(light1.size.width, light1.size.height);
      auto edge2 = std::minmax(light2.size.width, light2.size.height);
      auto lights_dis = std::sqrt((light1.center.x - light2.center.x) * (light1.center.x - light2.center.x) +
          (light1.center.y - light2.center.y) * (light1.center.y - light2.center.y));
      auto center_angle =
          std::atan(std::abs(light1.center.y - light2.center.y) / std::abs(light1.center.x - light2.center.x)) * 180 / CV_PI;
      center_angle = center_angle > 90 ? 180 - center_angle : center_angle;
      //std::cout << "center_angle: " << center_angle << std::endl;

      cv::RotatedRect rect;
      rect.angle = static_cast<float>(center_angle);
      rect.center.x = (light1.center.x + light2.center.x) / 2;
      rect.center.y = (light1.center.y + light2.center.y) / 2;
      float armor_width = std::abs(static_cast<float>(lights_dis) - std::max(edge1.first, edge2.first));
      float armor_height = std::max<float>(edge1.second, edge2.second);

      rect.size.width = std::max<float>(armor_width, armor_height);
      rect.size.height = std::min<float>(armor_width, armor_height);

      float light1_angle = light1.size.width < light1.size.height ? -light1.angle : light1.angle + 90;
      float light2_angle = light2.size.width < light2.size.height ? -light2.angle : light2.angle + 90;
      //std::cout << "light1_angle: " << light1_angle << std::endl;
      //std::cout << "light2_angle: " << light2_angle << std::endl;

      if (std::abs(light1_angle - light2_angle) < light_max_angle_diff_ &&
          std::max<float>(edge1.second, edge2.second)/std::min<float>(edge1.second, edge2.second) < 2.0 &&
          std::abs(center_angle) < armor_max_angle_ &&
          rect.size.width / (float) (rect.size.height) < armor_max_aspect_ratio_ &&
          std::abs(rect.size.area()) > armor_min_area_ &&
          gray_img_.at<uchar>(static_cast<int>(rect.center.y), static_cast<int>(rect.center.x))
              < armor_max_pixel_val_) {

        if (light1.center.x < light2.center.x) {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light1, light2);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_.DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        } else {
          std::vector<cv::Point2f> armor_points;
          CalcArmorInfo(armor_points, light2, light1);
          armors.emplace_back(ArmorInfo(rect, armor_points));
          if (enable_debug_)
            cv_toolbox_.DrawRotatedRect(show_armors_befor_filter_, rect, cv::Scalar(0, 255, 0), 2);
          armor_points.clear();
        }
      }
    }
  }
  if (enable_debug_)
    cv::imshow("armors_before_filter", show_armors_befor_filter_);
}

void ConstraintSet::FilterArmors(std::vector<ArmorInfo> &armors) {
  //std::cout << "********************************************FilterArmors********************************************" << std::endl;
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
    auto mean = mat_mean.at<double>(0, 0);
    //std::cout << "stddev: " << stddev << std::endl;
    //std::cout << "mean: " << mean << std::endl;

    if (stddev > armor_max_stddev_ || mean > armor_max_mean_) {
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
        if (armors[i].rect.angle > armors[j].rect.angle) {
          is_armor[i] = false;
          //std::cout << "i: " << i << std::endl;
        } else {
          is_armor[j] = false;
          //std::cout << "j: " << j << std::endl;
        }
      }
    }
  }
  //std::cout << armors.size() << std::endl;
  for (unsigned int i = 0; i < armors.size(); i++) {
    if (!is_armor[i]) {
      armors.erase(armors.begin() + i);
      is_armor.erase(is_armor.begin() + i);
      //std::cout << "index: " << i << std::endl;
    } else if (enable_debug_) {
      cv_toolbox_.DrawRotatedRect(show_armors_after_filter_, armors[i].rect, cv::Scalar(0, 255, 0), 2);
    }
  }
  if (enable_debug_)
    cv::imshow("armors_after_filter", show_armors_after_filter_);
}

ArmorInfo ConstraintSet::SlectFinalArmor(std::vector<ArmorInfo> &armors) {
  std::sort(armors.begin(),
            armors.end(),
            [](const ArmorInfo &p1, const ArmorInfo &p2) { return p1.rect.size.area() > p2.rect.size.area(); });

  return armors[0];
}

void ConstraintSet::CalcControlInfo(const ArmorInfo & armor,
                                    double &x,
                                    double &y,
                                    double &z,
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

  //z = camera_and_armor_diff_;

  //pitch = atan(z/optical_axis_offset_) + atan(4.8*(cameras_.GetCameraParam()[camera_id_].height_offset + armor.rect.center.y - 512)/1000./4.);
  //yaw   = atan(4.8*(640 - cameras_.GetCameraParam()[camera_id_].width_offset - armor.rect.center.x)/1000./4.);
  //yaw += yaw_offset_;
  //x = z/tan(pitch);
  //y = x*tan(yaw);
  //distance = sqrt(x*x + y*y + z*z);
  //if(!old_updated_) {
  //  old_x_ = x;
  //  old_y_ = y;
  //  old_z_ = z;

  //  old_distance_ = distance;
  //  old_pitch_    = pitch;
  //  old_yaw_      = yaw;
  //  old_updated_ = true;
  //}
  x = tvec.at<double>(2)/1000;
  y = -tvec.at<double>(0)/1000;
  z = tvec.at<double>(1)/1000;

  double fly_time = tvec.at<double>(2) / 1000.0 / bullet_speed;
  double gravity_offset = 0.5 * 0.98 * fly_time * fly_time * 1000;
  double xyz[3] = {tvec.at<double>(0), tvec.at<double>(1) + gravity_offset + gimbal_offset_z_, tvec.at<double>(2)};

  //calculate pitch
  pitch =  atan(xyz[1]/xyz[2]) + pitch_offset_;
  //calculate yaw
  yaw   = -atan2(xyz[0], xyz[2]) + yaw_offset_;

  distance = sqrt(tvec.at<double>(0)*tvec.at<double>(0) + tvec.at<double>(2)*tvec.at<double>(2))/1000.;

  if(!old_updated_) {
    old_x_ = x;
    old_y_ = y;
    old_z_ = z;

    old_distance_ = distance;
    old_pitch_    = pitch;
    old_yaw_      = yaw;
    old_updated_ = true;
  }

  SignalFilter(distance, old_distance_, filter_distance_count_, 0.3);
  SignalFilter(pitch, old_pitch_, filter_pitch_count_, 0.1);
  SignalFilter(yaw, old_yaw_, filter_yaw_count_, 0.1);

  SignalFilter(x, old_x_, filter_x_count_, 0.3);
  SignalFilter(y, old_y_, filter_y_count_, 0.3);
  //if(x > 3.5) x = 3.5;
  //if(yaws_in_moment_.size() < max_wait_fps_) {
  //  yaws_in_moment_.push_back(yaw);
  //} else {
  //  unsigned int count = 0;
  //  yaws_in_moment_.pop_front();
  //  yaws_in_moment_.push_back(yaw);
  //  double mean = std::accumulate(yaws_in_moment_.begin(), yaws_in_moment_.end(), 0.0)/(double)yaws_in_moment_.size();
  //  std::cout << "mean: " << mean << std::endl;
  //  for(auto &single_yaw: yaws_in_moment_) {
  //    count += std::fabs(single_yaw - mean) > min_pulse_angle_ ? 1 : 0;
  //  }
  //  std::cout << "count: " << count << std::endl;
  //  if(count > min_num_) {
  //    yaw = mean;
  //  }
  //}
}

void ConstraintSet::CalcArmorInfo(std::vector<cv::Point2f> &armor_points,
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

void ConstraintSet::SignalFilter(double &new_num, double &old_num, unsigned int &filter_count, double max_diff) {
  if(fabs(new_num - old_num) > max_diff && filter_count < 2) {
    filter_count++;
    new_num += max_diff;
  } else {
    filter_count = 0;
    old_num = new_num;
  }
}

ConstraintSet::~ConstraintSet() {
  cv_toolbox_.StopReadCamera();
  cap_.release();
}
} //namespace detection
} //namespace perception
} //namespace rrts
