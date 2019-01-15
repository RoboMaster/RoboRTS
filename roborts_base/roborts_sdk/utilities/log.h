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

#ifndef ROBORTS_SDK_LOG_H
#define ROBORTS_SDK_LOG_H
#include <iostream>
#include <fstream>
#include "glog/logging.h"
#include "glog/raw_logging.h"


#define LOG_INFO LOG(INFO)
#define LOG_WARNING LOG(WARNING)
#define LOG_ERROR LOG(ERROR)
#define LOG_FATAL LOG(FATAL)

#define LOG_INFO_IF(condition) LOG_IF(INFO,condition)
#define LOG_WARNING_IF(condition) LOG_IF(WARNING,condition)
#define LOG_ERROR_IF(condition) LOG_IF(ERROR,condition)
#define LOG_FATAL_IF(condition) LOG_IF(FATAL,condition)

#define LOG_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define LOG_WARNING_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define LOG_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define DLOG_INFO DLOG(INFO)
#define DLOG_WARNING DLOG(WARNING)
#define DLOG_ERROR DLOG(WARNING)

#define LOG_WARNING_FIRST_N(times) LOG_FIRST_N(WARNING, times)


class GLogWrapper {
 public:
  GLogWrapper(char* program) {
    google::InitGoogleLogging(program);
    FLAGS_stderrthreshold=google::WARNING;
    FLAGS_colorlogtostderr=true;
    FLAGS_v = 3;
    google::InstallFailureSignalHandler();
  }

  ~GLogWrapper()
  {
    google::ShutdownGoogleLogging();
  }
};


#endif //ROBORTS_SDK_LOG_H
