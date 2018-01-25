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
#include <random>

#include "common/example/example_algorithm.h"

namespace rrts{
namespace common {

ExampleAlgorithm::ExampleAlgorithm(int param) : ExampleBase(param) {
  LOG_INFO <<__FUNCTION__<< " contor!";
}

ExampleAlgorithm::~ExampleAlgorithm() {
  LOG_INFO <<__FUNCTION__<< " detor!";
}

ErrorInfo ExampleAlgorithm::Function() {

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()
  std::uniform_int_distribution<> dis(0, 10);

  int random = dis(gen);
  LOG_INFO << __FUNCTION__<<" running! random:"<< random;
  if(random == 1 ){
    return ErrorInfo(ErrorCode::Error);
  }
  return ErrorInfo(ErrorCode::OK);
}

}
}
