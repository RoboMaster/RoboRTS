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

#include "modules/driver/camera/proto/camera.pb.h"
#include "modules/driver/camera/camera_param.h"

#include "common/io.h"
#include "common/log.h"

namespace rrts {
namespace driver {
namespace camera {

CameraParam::CameraParam() {
  LoadCameraParam();
}

void CameraParam::LoadCameraParam() {
  rrts::drivers::Cameras camera_info;
  std::string file_name = "modules/driver/camera/config/camera.prototxt";
  bool read_state = rrts::common::ReadProtoFromTextFile(file_name, &camera_info);
  CHECK(read_state) << "Cannot open " << file_name;

  //camera number and ids
  auto camera_num = camera_info.camera().size();
  cameras_param_.resize(camera_num);
  for (unsigned int index = 0; index < camera_num; index++) {
    //camera id
    cameras_param_[index].mode = camera_info.camera(index).mode();
    cameras_param_[index].camera_id = (unsigned int) camera_info.camera(index).camera_id();
    cameras_param_[index].video_path = camera_info.camera(index).video_path();

    //camera resolution
    cameras_param_[index].resolution_width = camera_info.camera(index).resolution().width();
    cameras_param_[index].resolution_height = camera_info.camera(index).resolution().height();

    //camera matrix
    int camera_m_size = camera_info.camera(index).camera_matrix().data().size();
    float camera_m[camera_m_size];
    std::copy(camera_info.camera(index).camera_matrix().data().begin(),
              camera_info.camera(index).camera_matrix().data().end(),
              camera_m);
    cameras_param_[index].camera_matrix = cv::Mat(3, 3, CV_64F, camera_m);

    //camera distortion
    int rows = camera_info.camera(index).camera_distortion().data_size();
    float camera_dis[rows];
    std::copy(camera_info.camera(index).camera_distortion().data().begin(),
              camera_info.camera(index).camera_distortion().data().end(),
              camera_dis);
    cameras_param_[index].camera_distortion = cv::Mat(rows, 1, CV_64F, camera_dis);

  }
}
void CameraParam::GetCameraParam(std::vector<CameraInfo> &cameras_param) {
  cameras_param = cameras_param_;
}
std::vector<CameraInfo> CameraParam::GetCameraParam() {
  return cameras_param_;
}
} //namespace camera
} //namespace driver
} //namespace rrts