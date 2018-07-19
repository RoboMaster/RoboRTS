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

#ifndef DRIVERS_CAMERAPARAM_H
#define DRIVERS_CAMERAPARAM_H

#include <thread>
#include <string>
#include <opencv2/opencv.hpp>

namespace rrts {
namespace driver {
namespace camera {

struct CameraInfo {
  std::string camera_type;
  int mode;
  unsigned int camera_id;
  std::string video_path;
  cv::Mat camera_matrix;
  cv::Mat camera_distortion;

  unsigned int resolution_width;
  unsigned int resolution_height;

  unsigned int width_offset;
  unsigned int height_offset;

  unsigned int fps;
  bool auto_exposure;
  unsigned int exposure_value;
  unsigned int exposure_time;
  bool auto_white_balance;
  bool auto_gain;
  unsigned int contrast;


  cv::VideoCapture cap_handle;
};

class CameraParam {
 public:
  CameraParam();
  void LoadCameraParam();
  void GetCameraParam(std::vector<CameraInfo> &cameras_param);
  std::vector<CameraInfo> GetCameraParam();
  ~CameraParam() = default;
 private:
  std::vector<CameraInfo> cameras_param_;
};
} //namespace camera
} //namespace driver
} //namespace rrts

#endif //DRIVERS_CAMERAPARAM_H
