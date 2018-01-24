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

#ifndef COMMON_MAIN_INTERFACE_H
#define COMMON_MAIN_INTERFACE_H

#include "common/rrts.h"
#include "common/log.h"
#include <csignal>

/**
 * @brief Handle the keyboard interrupt
 * @param signal SIGINT signal
 */
void SignalHandler(int signal){
  if(ros::isInitialized() && ros::isStarted() && ros::ok() && !ros::isShuttingDown()){
    ros::shutdown();
  }
}

/**
 * @brief Macro definition for main function interface
 * @param RRTS_CLASS The class which inherits RRTS base-class interface
 */
#define MAIN(RRTS_CLASS,module_name)                                               \
  int main(int argc, char **argv){                                                 \
    rrts::common::GLogWrapper glog_wrapper(argv[0]);                               \
    signal(SIGINT, SignalHandler);                                                 \
    signal(SIGTERM,SignalHandler);                                                 \
    ros::init(argc, argv, module_name, ros::init_options::NoSigintHandler);        \
    RRTS_CLASS rrts(module_name);                                                  \
    rrts.Run();                                                                    \
    return 0;                                                                      \
  }

#endif //COMMON_MAIN_INTERFACE_H
