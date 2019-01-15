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

#include "log.h"
#include "sensor_odom.h"

namespace roborts_localization{

SensorOdom::SensorOdom(double alpha1,
                       double alpha2,
                       double alpha3,
                       double alpha4,
                       double alpha5) :
                       alpha1_(alpha1),
                       alpha2_(alpha2),
                       alpha3_(alpha3),
                       alpha4_(alpha4),
                       alpha5_(alpha5),
                       time_(0.0){
  odom_model_type_ = ODOM_MODEL_OMNI;
}

//TODO(kevin.li): This version only support omni model now. We will add diff model in the future.

void SensorOdom::SetModelOmni(double alpha1, double alpha2, double alpha3, double alpha4, double alpha5) {
  odom_model_type_ = ODOM_MODEL_OMNI;
  alpha1_ = alpha1;
  alpha2_ = alpha2;
  alpha3_ = alpha3;
  alpha4_ = alpha4;
  alpha5_ = alpha5;
}

bool SensorOdom::UpdateAction(SampleSetPtr sample_set_ptr, const SensorOdomData &odom_data) {

  DLOG_INFO << "Compute the new sample poses by motion model";

  Vec3d old_pose = (odom_data.pose) - (odom_data.delta);

  double delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;
  double delta_trans = std::sqrt(odom_data.delta[0] * odom_data.delta[0] + odom_data.delta[1] * odom_data.delta[1]);
  double delta_rot = odom_data.delta[2];

  double trans_hat_stddev = std::sqrt(alpha3_ * (delta_trans * delta_trans) + alpha1_ * (delta_rot * delta_rot));
  double rot_hat_stddev = std::sqrt(alpha4_ * (delta_rot * delta_rot) + alpha2_ * (delta_trans * delta_trans));
  double strafe_hat_stddev = std::sqrt(alpha1_ * (delta_rot * delta_rot) + alpha5_ * (delta_trans * delta_trans));

  for (int i = 0; i < sample_set_ptr->sample_count; i++) {

    delta_bearing = math::AngleDiff<double>(std::atan2(odom_data.delta(1),
                                               odom_data.delta(0)),
                                    old_pose(2)) + sample_set_ptr->samples_vec[i].pose(2);

    double cs_bearing = std::cos(delta_bearing);
    double sn_bearing = std::sin(delta_bearing);

    delta_trans_hat = delta_trans + math::RandomGaussianNumByStdDev<double>(trans_hat_stddev);
    delta_rot_hat = delta_rot + math::RandomGaussianNumByStdDev<double>(rot_hat_stddev);
    delta_strafe_hat = 0 + math::RandomGaussianNumByStdDev<double>(strafe_hat_stddev);

    sample_set_ptr->samples_vec[i].pose[0] += (delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing);
    sample_set_ptr->samples_vec[i].pose[1] += (delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing);
    sample_set_ptr->samples_vec[i].pose[2] += delta_rot_hat;

  }
  return true;
}

}// roborts_localization

