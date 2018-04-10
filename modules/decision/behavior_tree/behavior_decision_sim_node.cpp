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

#include "modules/decision/behavior_tree/behavior_tree.h"
#include "modules/decision/behavior_tree/planner_behavior.h"
#include "modules/decision/behavior_tree/condition_behavior.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "decision_node");
  auto blackboard_ptr = std::make_shared<rrts::decision::Blackboard>();
  auto planner_behavior_ptr = std::make_shared<rrts::decision::PlannerBehavior>(blackboard_ptr);
  auto enemy_behavior_ptr = std::make_shared<rrts::decision::EnemyBehavior>(blackboard_ptr, planner_behavior_ptr);
  auto patrol_behavior_ptr = std::make_shared<rrts::decision::PatrolBehavior>(blackboard_ptr, planner_behavior_ptr);

  auto selector_ptr = std::make_shared<rrts::decision::SelectorNode>("selector_behavior", blackboard_ptr);
  selector_ptr->AddChildren(enemy_behavior_ptr);
  selector_ptr->AddChildren(patrol_behavior_ptr);

  rrts::decision::BehaviorTree bt(selector_ptr,10000);
  bt.Execute();
}
