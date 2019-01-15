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

#ifndef ROBORTS_COMMON_ERROR_CODE_H
#define ROBORTS_COMMON_ERROR_CODE_H

#include <string>

namespace roborts_common{

enum ErrorCode{
  OK = 0,
  Error = 1,
/************************HARDWARE********************/



/***********************SOFTWARRE********************/
  /***************DRIVER******************/
    //camera
    CAMERA_ERROR = 10000,
    IMAGE_READ_ERROR = 10001,
    STOP_DETECTION = 10002,

    //lidar
    LIDAR_ERROR = 10100,


  /**************PERCEPTION***************/
    //mapping
    MAPPING_ERROR = 11000,

    //map
    MAP_ERROR = 12000,

    //localization
    LOCALIZATION_INIT_ERROR = 12100,

    //detection
    DETECTION_INIT_ERROR = 12200,


  /**************DECISION*****************/
    //decision
    DECISION_ERROR = 13000,


  /**************PLANNING*****************/

    //global planner
    GP_INITILIZATION_ERROR = 14000,
    GP_GET_POSE_ERROR,
    GP_POSE_TRANSFORM_ERROR,
    GP_GOAL_INVALID_ERROR,
    GP_PATH_SEARCH_ERROR,
    GP_MOVE_COST_ERROR,
    GP_MAX_RETRIES_FAILURE,
    GP_TIME_OUT_ERROR,


    //local planner
    LP_PLANNING_ERROR = 14100,
    LP_INITILIZATION_ERROR = 14101,
    LP_ALGORITHM_INITILIZATION_ERROR = 14102,
    LP_ALGORITHM_TRAJECTORY_ERROR= 14103,
    LP_ALGORITHM_GOAL_REACHED= 14104,
    LP_MAX_ERROR_FAILURE = 14105,
    LP_PLANTRANSFORM_ERROR = 14106,
    LP_OPTIMAL_ERROR = 14107,
    LP_VELOCITY_ERROR = 14108,
    LP_OSCILLATION_ERROR


  /**************CONTROL******************/
};

class ErrorInfo {

 public:
  ErrorInfo():error_code_(ErrorCode::OK),error_msg_(""){};
  ErrorInfo(ErrorCode error_code, const std::string &error_msg=""):error_code_(error_code),error_msg_(error_msg){};

  ErrorInfo& operator= ( const ErrorInfo& error_info ) {
    if (&error_info != this) {
      error_code_ = error_info.error_code_;
      error_msg_ = error_info.error_msg_;
    }
    return *this;
  }

  static ErrorInfo OK(){
    return ErrorInfo();
  }

  ErrorCode error_code() const { return error_code_;};
  const std::string &error_msg() const { return error_msg_; }

  bool operator==(const ErrorInfo& rhs){
    return error_code_==rhs.error_code();
  }

  bool IsOK() const{
    return (error_code_ == ErrorCode::OK);
  }

  ~ErrorInfo()= default;

 private:
  ErrorCode error_code_;
  std::string error_msg_;


};

} //namespace roborts_common

#endif //ROBORTS_COMMON_ERROR_CODE_H
