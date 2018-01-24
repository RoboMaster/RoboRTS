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

#ifndef COMMON_RRTS_H
#define COMMON_RRTS_H
#include <string>
#include <ros/ros.h>

namespace rrts {
namespace common {
/**
 * @brief The base class which runs in main function interface.
 * It should be inherited by each module
 */
class RRTS {
 public:
  /**
   * @brief Constructor function to set the thread number of callback queue and the module name.
   * @param thread_num the thread number of callback queue
   * @param name the module name
   */
  RRTS(std::string name,uint32_t thread_num = 1);

  /**
   * @brief Start to run the all the callbacks in the queue
   * @note The inheriting class is not recommendded to override this function which may cause a chaos of callback process
   */
  virtual void Run();
  /**
   * @brief Destructor function
   */
  virtual ~RRTS() = default;

 protected:

  std::string name_;
  uint32_t thread_num_;

};

} //namespace common
} //namespace rrts
#endif //COMMON_RRTS_H
