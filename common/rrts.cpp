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

#include "common/rrts.h"


namespace rrts {
namespace common {

  RRTS::RRTS( std::string name,uint32_t thread_num):name_(name),thread_num_(thread_num) {}

  void RRTS::Run() {
    ros::AsyncSpinner async_spinner(thread_num_);
    async_spinner.start();
    ros::waitForShutdown();
  }


} //namespace common
} //namespace rrts

