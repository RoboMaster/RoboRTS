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

/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H
#define MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H

#include "modules/perception/localization/amcl/sensors/sensor_base.h"
#include "modules/perception/localization/amcl/math/math.h"
#include "modules/perception/localization/amcl/proto/amcl.pb.h"

namespace rrts {
namespace perception {
namespace localization {

class SensorOdomData : public SensorData {
 public:
  math::Vec3d pose;
  math::Vec3d delta;
};

class SensorOdom : public SensorBase {
 public:
  /**
   * @brief Default constructor
   */
  SensorOdom();


  /**
   * @brief Set odometry model to omni model.
   * @param alpha1
   * @param alpha2
   * @param alpha3
   * @param alpha4
   * @param alpha5
   */
  void SetModelOmni(double alpha1,
                    double alpha2,
                    double alpha3,
                    double alpha4,
                    double alpha5);

  /**
   * @brief Update the filter based on the action model
   * @param pf_ptr Particle filter object pointer
   * @param sensor_data_ptr Sensor data object pointer
   * @return Returns true if the filter has been updated
   */
  virtual bool UpdateAction(ParticleFilterPtr pf_ptr,
                            SensorData *sensor_data_ptr);

 private:
  /**
   * @brief Current data timestamp
   */
  double time_;

  /**
   * @brief Model type
   */
  OdomModel odom_model_type_;

  /**
   * @brief Drift parameters
   */
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};

}
}
}


#endif //MODULE_PERCEPTION_PLANNING_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H
