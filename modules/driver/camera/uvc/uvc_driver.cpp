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

#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include "modules/driver/camera/uvc/uvc_driver.h"
#include "common/main_interface.h"

namespace rrts {
namespace driver {
namespace camera {
UVCDriver::UVCDriver() {
  LoadParam();
  read_camera_initialized_ = false;
}
void UVCDriver::LoadParam() {
  camera_param_.GetCameraParam(cameras_);
}
/**
 * @brief
 */
void UVCDriver::Init(unsigned int camera_num) {
  if (cameras_[camera_num].mode == 0) {
    unsigned int camera_id = cameras_[camera_num].camera_id;
    cameras_[camera_num].cap_handle.open(camera_id);
    SetCameraExposure(std::to_string(camera_id), cameras_[camera_num].exposure_value);
    CHECK(cameras_[camera_num].cap_handle.isOpened()) << "Cannot open camera " << camera_id << ".";
    read_camera_initialized_ = true;
  }
  else {
    cameras_[camera_num].cap_handle.open(cameras_[camera_num].video_path);
    SetCameraExposure(cameras_[camera_num].video_path, cameras_[camera_num].exposure_value);
    CHECK(cameras_[camera_num].cap_handle.isOpened()) << "Cannot open " << cameras_[camera_num].video_path << ".";
    read_camera_initialized_ = true;
  }
}
void UVCDriver::StartReadCamera(unsigned int camera_num, cv::Mat &img) {
  if(!read_camera_initialized_)
    Init(camera_num);
  else {
    cameras_[camera_num].cap_handle >> img;
    img = img(cv::Rect(cameras_[camera_num].width_offset, 
                       cameras_[camera_num].height_offset,
                       cameras_[camera_num].resolution_width, 
                       cameras_[camera_num].resolution_height));
  }
}
/**
 * @brief Stop to read image.
 */
void UVCDriver::StopReadCamera() {
}

void UVCDriver::SetCameraExposure(std::string id, int val)
{
  int cam_fd;
  if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
    std::cerr << "Camera open error" << std::endl;
  }

  struct v4l2_control control_s;
  control_s.id = V4L2_CID_EXPOSURE_AUTO;
  control_s.value = V4L2_EXPOSURE_MANUAL;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  // Set exposure value
  control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  control_s.value = val;
  ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
  close(cam_fd);
}

UVCDriver::~UVCDriver() {
}
} //namespace camera
} //namespace drivers
} //namespace rrts
