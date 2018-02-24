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

#ifndef MODULES_PERCEPTION_DETECTION_DETECTION_ARMOR_DETECTION_BASE_H
#define MODULES_PERCEPTION_DETECTION_DETECTION_ARMOR_DETECTION_BASE_H

#include <vector>
#include "common/error_code.h"

namespace rrts{
namespace perception {
namespace detection {

using rrts::common::ErrorInfo;

class ArmorDetectionBase {
 public:
  ArmorDetectionBase();
  virtual void LoadParam();
  virtual ErrorInfo DetectArmor(double &distance, double &pitch, double &yaw) = 0;
  virtual ~ArmorDetectionBase() = default;
};
} //namespace detection
} //namespace perception
} //namespace rrts

#endif //MODULES_PERCEPTION_DETECTION_DETECTION_ARMOR_DETECTION_BASE_H