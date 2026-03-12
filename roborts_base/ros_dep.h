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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

/**
 * @file ros_dep.h
 * @brief Centralized ROS message and service includes for roborts_base.
 *
 * This header aggregates all ROS message dependencies used by the
 * chassis, gimbal, and referee system modules to avoid redundant includes.
 */

#ifndef ROBORTS_BASE_ROS_DEP_H
#define ROBORTS_BASE_ROS_DEP_H

// ROS core
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Standard ROS messages
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

// ---- Chassis Messages ----
#include "roborts_msgs/TwistAccel.h"

// ---- Gimbal Messages ----
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalRate.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"

// ---- Referee System Messages ----
#include "roborts_msgs/BonusStatus.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/ProjectileSupply.h"
#include "roborts_msgs/RobotBonus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/SupplierStatus.h"

#endif // ROBORTS_BASE_ROS_DEP_H
