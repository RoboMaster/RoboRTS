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

#ifndef TIMER_H
#define TIMER_H
#include <stdio.h>
#include <boost/timer.hpp>


#define TIMER_START(FUNC) boost::timer t_##FUNC;
#define TIMER_END(FUNC) std::cout << "[" << #FUNC << "]" << "cost time: " << t_##FUNC.elapsed() << std::endl;

#endif // TIMER_H
