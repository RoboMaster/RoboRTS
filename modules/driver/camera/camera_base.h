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

#ifndef MODULES_DRIVERS_CAMERA_CAMERA_BASE_H
#define MODULES_DRIVERS_CAMERA_CAMERA_BASE_H

#include <vector>
#include <opencv2/opencv.hpp>
#include "common/error_code.h"

namespace rrts{
namespace driver {
namespace camera {

using rrts::common::ErrorInfo;

class CameraBase {
 public:
  CameraBase();
  virtual void LoadParam();
  virtual void StartReadCamera(unsigned int camera_num, cv::Mat &img) = 0;
  virtual ~CameraBase() = default;
};
} //namespace camera
} //namespace driver
} //namespace rrts

#endif //MODULES_DRIVERS_CAMERA_CAMERA_BASE_H