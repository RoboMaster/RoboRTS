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

#include "common/log.h"

#include "modules/perception/localization/amcl/particle_filter/particle_filter_gaussian_pdf.h"
#include "modules/perception/localization/amcl/sensors/sensor_odom.h"


namespace rrts {
namespace perception {
namespace localization {


SensorOdom::SensorOdom() {
  this->time_ = 0.0;
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

bool SensorOdom::UpdateAction(ParticleFilterPtr pf_ptr, SensorData *sensor_data_ptr) {

  auto ndata_ptr = (SensorOdomData *) sensor_data_ptr;
  DLOG_INFO << "Compute the new sample poses";
  auto pf_sample_set = pf_ptr->sample_set_ptr_array_[pf_ptr->current_set_];

  math::Vec3d old_pose = (ndata_ptr->pose) - (ndata_ptr->delta);

//TODO(kevin.li): Other odom model type support

  double delta_trans, delta_rot, delta_bearing;
  double delta_trans_hat, delta_rot_hat, delta_strafe_hat;
  delta_trans = std::sqrt(ndata_ptr->delta[0] * ndata_ptr->delta[0] + ndata_ptr->delta[1] * ndata_ptr->delta[1]);
  delta_rot = ndata_ptr->delta[2];

  double trans_hat_stddev = std::sqrt(alpha3_ * (delta_trans * delta_trans) + alpha1_ * (delta_rot * delta_rot));
  double rot_hat_stddev = std::sqrt(alpha4_ * (delta_rot * delta_rot) + alpha2_ * (delta_trans * delta_trans));
  double strafe_hat_stddev = std::sqrt(alpha1_ * (delta_rot * delta_rot) + alpha5_ * (delta_trans * delta_trans));

  ParticleFilterGaussianPdf pf_gauss_pdf;

  for (int i = 0; i < pf_sample_set->sample_count; i++) {

    delta_bearing = math::AngleDiff<double>(std::atan2(ndata_ptr->delta(1),
                                               ndata_ptr->delta(0)),
                                    old_pose(2)) + pf_sample_set->samples_vec[i].pose(2);

    double cs_bearing = std::cos(delta_bearing);
    double sn_bearing = std::sin(delta_bearing);

    delta_trans_hat = delta_trans + math::RandomGaussianNumByStdDev<double>(trans_hat_stddev);
    delta_rot_hat = delta_rot + math::RandomGaussianNumByStdDev<double>(rot_hat_stddev);
    delta_strafe_hat = 0 + math::RandomGaussianNumByStdDev<double>(strafe_hat_stddev);

    pf_sample_set->samples_vec[i].pose[0] += (delta_trans_hat * cs_bearing + delta_strafe_hat * sn_bearing);
    pf_sample_set->samples_vec[i].pose[1] += (delta_trans_hat * sn_bearing - delta_strafe_hat * cs_bearing);
    pf_sample_set->samples_vec[i].pose[2] += delta_rot_hat;

  }
  return true;
}

}
}
}

