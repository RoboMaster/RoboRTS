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

#ifndef ROBORTS_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H
#define ROBORTS_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H

#include "../particle_filter/particle_filter.h"
#include "localization_math.h"

namespace roborts_localization{

enum OdomModel{
  ODOM_MODEL_DIFF = 0,
  ODOM_MODEL_OMNI = 1
};


class SensorOdomData{
 public:
  Vec3d pose;
  Vec3d delta;
};

class SensorOdom {
 public:
  /**
   * @brief Default constructor
   */
  SensorOdom(double alpha1,
             double alpha2,
             double alpha3,
             double alpha4,
             double alpha5);

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
  bool UpdateAction(SampleSetPtr pf_sample_set_ptr,
                    const SensorOdomData &odom_data);

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

}// roborts_localization

#endif //ROBORTS_LOCALIZATION_AMCL_SENSORS_SENSOR_ODOM_H
