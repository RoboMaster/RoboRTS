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

#include "camera_param.h"
#include "proto/camera_param.pb.h"

#include "io/io.h"

namespace roborts_camera{

CameraParam::CameraParam() {
  LoadCameraParam();
}

void CameraParam::LoadCameraParam() {
  Cameras camera_info;
  std::string file_name = ros::package::getPath("roborts_camera") + "/config/camera_param.prototxt";
  bool read_state = roborts_common::ReadProtoFromTextFile(file_name, &camera_info);
  ROS_ASSERT_MSG(read_state, "Cannot open %s", file_name.c_str());

  //camera number and ids
  int camera_num = camera_info.camera().size();
  cameras_param_.resize(camera_num);
  for(unsigned int index = 0; index < camera_num; index++) {

    //camera id
    cameras_param_[index].camera_name = camera_info.camera(index).camera_name();
    cameras_param_[index].camera_type = camera_info.camera(index).camera_type();
    cameras_param_[index].camera_path = camera_info.camera(index).camera_path();

    //camera resolution
    cameras_param_[index].resolution_width = camera_info.camera(index).resolution().width();
    cameras_param_[index].resolution_height = camera_info.camera(index).resolution().height();
    cameras_param_[index].width_offset = camera_info.camera(index).resolution().width_offset();
    cameras_param_[index].height_offset = camera_info.camera(index).resolution().height_offset();

    //fps
    cameras_param_[index].fps = camera_info.camera(index).fps();

    //exposure
    cameras_param_[index].auto_exposure = camera_info.camera(index).auto_exposure();
    cameras_param_[index].exposure_value = camera_info.camera(index).exposure_value();
    cameras_param_[index].exposure_time = camera_info.camera(index).exposure_time();

    //white_balance and gain
    cameras_param_[index].auto_white_balance = camera_info.camera(index).auto_white_balance();
    cameras_param_[index].auto_gain = camera_info.camera(index).auto_gain();

    //constrast
    cameras_param_[index].contrast = camera_info.camera(index).contrast();

    //camera matrix
    int camera_m_size = camera_info.camera(index).camera_matrix().data().size();
    double camera_m[camera_m_size];
    std::copy(camera_info.camera(index).camera_matrix().data().begin(),
              camera_info.camera(index).camera_matrix().data().end(),
              camera_m);
    cameras_param_[index].camera_matrix = cv::Mat(3, 3, CV_64F, camera_m).clone();

    //camera distortion
    int rows = camera_info.camera(index).camera_distortion().data_size();
    double camera_dis[rows];
    std::copy(camera_info.camera(index).camera_distortion().data().begin(),
              camera_info.camera(index).camera_distortion().data().end(),
              camera_dis);
    cameras_param_[index].camera_distortion = cv::Mat(rows, 1, CV_64F, camera_dis).clone();

    //ros camera info
    cameras_param_[index].ros_camera_info = boost::make_shared<sensor_msgs::CameraInfo>();
    cameras_param_[index].ros_camera_info->header.frame_id = cameras_param_[index].camera_name;
    cameras_param_[index].ros_camera_info->width = cameras_param_[index].resolution_width;
    cameras_param_[index].ros_camera_info->height = cameras_param_[index].resolution_height;
    cameras_param_[index].ros_camera_info->D = cameras_param_[index].camera_distortion;
    cameras_param_[index].ros_camera_info->roi.x_offset = cameras_param_[index].width_offset;
    cameras_param_[index].ros_camera_info->roi.y_offset = cameras_param_[index].height_offset;
    std::copy(camera_m, camera_m + camera_m_size, cameras_param_[index].ros_camera_info->K.begin());

  }
}
std::vector<CameraInfo>& CameraParam::GetCameraParam() {
  return cameras_param_;
}

} //namespace roborts_camera
