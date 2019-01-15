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

#ifndef ROBORTS_CAMERA_CAMERA_PARAM_H
#define ROBORTS_CAMERA_CAMERA_PARAM_H

#include <thread>
#include <string>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>

namespace roborts_camera {
/**
 * @brief Information and parameter of camera
 */
struct CameraInfo {
  //! name of camera
  std::string camera_name;
  //! type of camera for camera factory product instantiation
  std::string camera_type;
  //! path of camera, i.e. /dev/video0
  std::string camera_path;
  //! camera matrix
  cv::Mat camera_matrix;
  //! camera distortion matrix
  cv::Mat camera_distortion;

  //! resolution width
  unsigned int resolution_width;
  //! resolution height
  unsigned int resolution_height;

  //! width offset for image crop
  unsigned int width_offset;
  //! height offset for image crop
  unsigned int height_offset;

  //! camera fps
  unsigned int fps;
  //! flag of auto exposure
  bool auto_exposure;
  //! exposure value
  unsigned int exposure_value;
  //! exposure time
  unsigned int exposure_time;
  //! auto white balance
  bool auto_white_balance;
  //! auto gain
  bool auto_gain;
  //! contrast
  unsigned int contrast;

  //! camera information in form of ROS sensor_msgs
  sensor_msgs::CameraInfoPtr ros_camera_info;
  //! opencv video capture
  cv::VideoCapture cap_handle;
};

/**
 * @brief camera parameter class to load and get camera parameters
 */
class CameraParam {
 public:
  /**
   * @brief Constructor of CameraParam, load the camera parameter
   */
  CameraParam();
  /**
   * @brief load all camera parameters for different cameras
   */
  void LoadCameraParam();
  /**
   * @brief Get the camera parameters for different cameras
   * @param cameras_param Camera parameters to update for different cameras
   */
  void GetCameraParam(std::vector<CameraInfo> &cameras_param);
  /**
   * @brief Get the camera parameters for different cameras
   * @return Updated Camera parameters for different cameras
   */
  std::vector<CameraInfo>& GetCameraParam();
  ~CameraParam() = default;
 private:
  //! camera parameters for different cameras
  std::vector<CameraInfo> cameras_param_;
};

} //namespace roborts_camera

#endif //ROBORTS_CAMERA_CAMERA_PARAM_H
