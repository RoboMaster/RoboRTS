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
#include "modules/decision/ICRA_decision/action_behavior.h"

#include "common/log.h"

int main(int argc, char **argv){
  google::InitGoogleLogging(argv[0]);
  FLAGS_stderrthreshold = google::WARNING;
  FLAGS_colorlogtostderr = true;
  FLAGS_v = 3;
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "decision_node");
  /*
   *
   *
   *                                                        robot_selector
   *                                                   /                      \
   *                                 robot_1_condition                          game_status_selector
   *                                        |                                 /                      \
   *                                  robot_1_selector          game_stop_con                      game_start_selector
   *                                 /     |     |     \              |
   *                                r      d     c      T          wait_ac
   *                                |      |     |
   *                               AA     SH    TCD
   *                                                                                              /                    \
   *                                                                                 obt_buff_con                         without_buff_selector
   *                                                                                      |                             /                         \
   *                                                                             offensive_selector             emy_buff_con                      search_buff_selector
   *                                                                         /    |    |    |    |  |   \              |                          /                        \
   *                                                                      dmp     d    a    c    r  SR   P        emy_buff_sel              rfid_d_con                       rfid_active_sel
   *                                                                       |      |    |    |    |           /    /   |   |   |   \              |                           /          \
   *                                                                       E     CH   TWA  TCD  AA         dmp    d   a   c   r   T    plan_buff_selector               detect           c
   *                                                                                                        |     |    |   |   |        /        |         \                |             |
   *                                                                                                        E    CH   TWA TCD AA       d         a          GB             SH            TCD
   *                                                                                                                                   |         |
   *                                                                                                                                  CH        TWA
   *
   */
  auto blackboard_ptr_ = std::make_shared<rrts::decision::Blackboard>("/modules/decision/vic_decision/config/decision.prototxt");

  auto goal_factory_ = std::make_shared<rrts::decision::GoalFactory>(blackboard_ptr_,
                                                                     "/modules/decision/vic_decision/config/decision.prototxt");

  rrts::decision::DecisionConfig robot_config;
  rrts::common::ReadProtoFromTextFile("/modules/decision/vic_decision/config/decision.prototxt", &robot_config);

  //action
  auto shoot_action_ = std::make_shared<rrts::decision::ShootAction>(blackboard_ptr_, goal_factory_);
  auto chase_action_ = std::make_shared<rrts::decision::ChaseAction>(blackboard_ptr_, goal_factory_);
  auto gain_buff_action_ = std::make_shared<rrts::decision::GainBuffAction>(blackboard_ptr_, goal_factory_);
  auto turn_to_hurt_action_ = std::make_shared<rrts::decision::TurnToWoundedArmorAction>(blackboard_ptr_,
                                                                                         goal_factory_);
  auto escape_action_ = std::make_shared<rrts::decision::EscapeAction>(blackboard_ptr_, goal_factory_);
  auto color_detected_action_ = std::make_shared<rrts::decision::TurnToDetectedDirection>(blackboard_ptr_, goal_factory_);

  auto patrol_action_ = std::make_shared<rrts::decision::PatrolAction>(blackboard_ptr_, goal_factory_);
  auto search_action_ = std::make_shared<rrts::decision::SearchAction>(blackboard_ptr_, goal_factory_);
  auto wait_action_ = std::make_shared<rrts::decision::WaitAction>(blackboard_ptr_, goal_factory_);
  auto whirl_action_ = std::make_shared<rrts::decision::WhirlAction>(blackboard_ptr_, goal_factory_);
  auto auxiliary_action = std::make_shared<rrts::decision::AuxiliaryAction>(blackboard_ptr_, goal_factory_);
  auto wing_auxiliary_action = std::make_shared<rrts::decision::AuxiliaryAction>(blackboard_ptr_, goal_factory_);


  //detected and swinging shoot

  auto detect_shoot_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid active detect shoot condition",
                                                                                    blackboard_ptr_,
                                                                                    shoot_action_,
                                                                                    [&]() {
                                                                                      if (blackboard_ptr_->GetEnemyDetected()) {
                                                                                        return true;
                                                                                      } else {
                                                                                        return false;
                                                                                      }
                                                                                    } ,
                                                                                    rrts::decision::AbortType::BOTH);

  auto rfid_active_detected_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid active detected condition",
                                                                                            blackboard_ptr_,
                                                                                            color_detected_action_,
                                                                                            [&]() {
                                                                                              if (blackboard_ptr_->GetColordetected()
                                                                                                  != rrts::decision::ColorDetected ::NONE
                                                                                                  && blackboard_ptr_->GetColordetected()
                                                                                                      != rrts::decision::ColorDetected ::FRONT) {
                                                                                                return true;
                                                                                              } else {
                                                                                                return false;
                                                                                              }
                                                                                            },
                                                                                            rrts::decision::AbortType::LOW_PRIORITY);

  //detected and chasing shoot

  auto detect_enemy_condition_ = std::make_shared<rrts::decision::PreconditionNode>("plan buff detect enemy condition",
                                                                                    blackboard_ptr_,
                                                                                    chase_action_,
                                                                                    [&]() {
                                                                                      if (blackboard_ptr_->GetEnemyDetected()) {
                                                                                        return true;
                                                                                      } else {
                                                                                        return false;
                                                                                      }
                                                                                    },
                                                                                    rrts::decision::AbortType::BOTH);

  //plan_buff_selector


  auto under_attack_condition_ = std::make_shared<rrts::decision::PreconditionNode>("plan buff under attack condition",
                                                                                    blackboard_ptr_,
                                                                                    turn_to_hurt_action_,
                                                                                    [&](){
                                                                                      if (blackboard_ptr_->GetArmorAttacked()
                                                                                          != rrts::decision::ArmorAttacked::NONE
                                                                                          && blackboard_ptr_->GetArmorAttacked()
                                                                                              != rrts::decision::ArmorAttacked::FRONT) {
                                                                                        return true;
                                                                                      } else {
                                                                                        return false;
                                                                                      }
                                                                                    }, rrts::decision::AbortType::LOW_PRIORITY);

  auto plan_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("plan buff selector", blackboard_ptr_);
  plan_buff_selector_->AddChildren(detect_enemy_condition_);
  plan_buff_selector_->AddChildren(under_attack_condition_);
  plan_buff_selector_->AddChildren(gain_buff_action_);

  // search_buff_selector
  auto rfid_condition_ = std::make_shared<rrts::decision::PreconditionNode>("rfid condition_", blackboard_ptr_,
                                                                            plan_buff_selector_,
                                                                            [&](){
                                                                              if (blackboard_ptr_->GetRfidActive()) {
                                                                                return false;
                                                                              } else {
                                                                                return true;
                                                                              }
                                                                            },
                                                                            rrts::decision::AbortType::BOTH);

  auto search_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("gain buff selector", blackboard_ptr_);

  search_buff_selector_->AddChildren(rfid_condition_);
  search_buff_selector_->AddChildren(detect_shoot_condition_);

  //enemy_buff_selector


  auto emy_buff_dmp_condition_ = std::make_shared<rrts::decision::PreconditionNode>("enemy dmp condition",
                                                                                    blackboard_ptr_,
                                                                                    escape_action_,
                                                                                    [&](){
                                                                                      if (blackboard_ptr_->HurtedPerSecond() > 600 
                                                                                      || blackboard_ptr_->GetSentBulletStatus()) {
                                                                                        return true;
                                                                                      } else {
                                                                                        return false;
                                                                                      }
                                                                                    },
                                                                                    rrts::decision::AbortType::LOW_PRIORITY);

  auto inferior_detect_enemy_condition_ = std::make_shared<rrts::decision::PreconditionNode>("inferior detect enemy condition",
                                                                                             blackboard_ptr_,
                                                                                             shoot_action_,
                                                                                             [&]() {
                                                                                               if (blackboard_ptr_->GetEnemyDetected()) {
                                                                                                 return true;
                                                                                               } else {
                                                                                                 return false;
                                                                                               }
                                                                                             }, rrts::decision::AbortType::BOTH);


  auto inferior_detected_condition_ = std::make_shared<rrts::decision::PreconditionNode>("inferior detected condition",
                                                                                         blackboard_ptr_,
                                                                                         color_detected_action_,
                                                                                         [&]() {
                                                                                           if (blackboard_ptr_->GetColordetected()
                                                                                               != rrts::decision::ColorDetected ::NONE
                                                                                               && blackboard_ptr_->GetColordetected()
                                                                                                   != rrts::decision::ColorDetected ::FRONT) {
                                                                                             return true;
                                                                                           } else {
                                                                                             return false;
                                                                                           }
                                                                                         }, rrts::decision::AbortType::LOW_PRIORITY);

  auto emy_buff_attack_condition_ = std::make_shared<rrts::decision::PreconditionNode>("under enemy buff attack condition",
                                                                                       blackboard_ptr_,
                                                                                       turn_to_hurt_action_,
                                                                                       [&](){
                                                                                         if (blackboard_ptr_->GetArmorAttacked()
                                                                                             != rrts::decision::ArmorAttacked::NONE
                                                                                             && blackboard_ptr_->GetArmorAttacked()
                                                                                                 != rrts::decision::ArmorAttacked::FRONT) {
                                                                                           return true;
                                                                                         } else {
                                                                                           return false;
                                                                                         }
                                                                                       },
                                                                                       rrts::decision::AbortType::LOW_PRIORITY);

  auto master_inferior_receive_condition = std::make_shared<rrts::decision::PreconditionNode>("master inferior receive condition",
                                                                                              blackboard_ptr_,
                                                                                              auxiliary_action,
                                                                                              [&]() {
                                                                                                if (blackboard_ptr_->GetAuxiliaryState()) {
                                                                                                  return true;
                                                                                                } else {
                                                                                                  return false;
                                                                                                }
                                                                                              }, rrts::decision::AbortType::LOW_PRIORITY);



  auto enemy_obtain_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("enemy obtain buff selector", blackboard_ptr_);
  enemy_obtain_buff_selector_->AddChildren(emy_buff_dmp_condition_);
  enemy_obtain_buff_selector_->AddChildren(inferior_detect_enemy_condition_);
  enemy_obtain_buff_selector_->AddChildren(inferior_detected_condition_);
  enemy_obtain_buff_selector_->AddChildren(emy_buff_attack_condition_);
  enemy_obtain_buff_selector_->AddChildren(whirl_action_);

  //without_buff_selector
  auto enemy_obtain_buff_condition_ = std::make_shared<rrts::decision::PreconditionNode>("enemy obtain buff", blackboard_ptr_,
                                                                                         enemy_obtain_buff_selector_,
                                                                                         [&](){
                                                                                           if (blackboard_ptr_->GetBuffStatus()
                                                                                               == rrts::decision::BuffStatus::ENEMY) {
                                                                                             return true;
                                                                                           } else {
                                                                                             return false;
                                                                                           }
                                                                                         },
                                                                                         rrts::decision::AbortType::BOTH);
  auto without_buff_selector_ = std::make_shared<rrts::decision::SelectorNode>("without_buff_selector", blackboard_ptr_);
  without_buff_selector_->AddChildren(enemy_obtain_buff_condition_);
  without_buff_selector_->AddChildren(search_buff_selector_);


  //offensive_selector
  auto offensive_dmp_condition_ = std::make_shared<rrts::decision::PreconditionNode>("offensive dmp condition",
                                                                                     blackboard_ptr_,
                                                                                     escape_action_,
                                                                                     [&](){
                                                                                       if (blackboard_ptr_->HurtedPerSecond() > 400 
                                                                                       || blackboard_ptr_->GetSentBulletStatus()) {
                                                                                         return true;
                                                                                       } else {
                                                                                         return false;
                                                                                       }
                                                                                     },
                                                                                     rrts::decision::AbortType::LOW_PRIORITY);

  auto offensive_detect_enemy_condition_ = std::make_shared<rrts::decision::PreconditionNode>("offensive_detect_enemy_condition",
                                                                                              blackboard_ptr_,
                                                                                              chase_action_,
                                                                                              [&]() {
                                                                                                if (blackboard_ptr_->GetEnemyDetected()) {
                                                                                                  return true;
                                                                                                } else {
                                                                                                  return false;
                                                                                                }
                                                                                              },
                                                                                              rrts::decision::AbortType::BOTH);

  auto offensive_under_attack_condition_ = std::make_shared<rrts::decision::PreconditionNode>("offensive_under_attack_condition",
                                                                                              blackboard_ptr_,
                                                                                              turn_to_hurt_action_,
                                                                                              [&](){
                                                                                                if (blackboard_ptr_->GetArmorAttacked()
                                                                                                    != rrts::decision::ArmorAttacked::NONE
                                                                                                    && blackboard_ptr_->GetArmorAttacked()
                                                                                                        != rrts::decision::ArmorAttacked::FRONT) {
                                                                                                  return true;
                                                                                                } else {
                                                                                                  return false;
                                                                                                }
                                                                                              },
                                                                                              rrts::decision::AbortType::LOW_PRIORITY);

  auto offensive_detected_condition_ = std::make_shared<rrts::decision::PreconditionNode>("offensive detected condition",
                                                                                          blackboard_ptr_,
                                                                                          color_detected_action_,
                                                                                          [&]() {
                                                                                            if (blackboard_ptr_->GetColordetected()
                                                                                                != rrts::decision::ColorDetected ::NONE
                                                                                                && blackboard_ptr_->GetColordetected()
                                                                                                    != rrts::decision::ColorDetected ::FRONT) {
                                                                                              return true;
                                                                                            } else {
                                                                                              return false;
                                                                                            }
                                                                                          },
                                                                                          rrts::decision::AbortType::LOW_PRIORITY);

  auto master_receive_condition = std::make_shared<rrts::decision::PreconditionNode>("master receive condition", blackboard_ptr_,
                                                                                     auxiliary_action,
                                                                                     [&](){
                                                                                       if (blackboard_ptr_->GetAuxiliaryState()) {
                                                                                         return true;
                                                                                       } else {
                                                                                         return false;
                                                                                       }
                                                                                     },
                                                                                     rrts::decision::AbortType::LOW_PRIORITY);



  auto offensive_selector_ = std::make_shared<rrts::decision::SelectorNode>("offensive_selector", blackboard_ptr_);

  offensive_selector_->AddChildren(offensive_dmp_condition_);
  offensive_selector_->AddChildren(offensive_detect_enemy_condition_);
  offensive_selector_->AddChildren(offensive_under_attack_condition_);
  offensive_selector_->AddChildren(offensive_detected_condition_);
  offensive_selector_->AddChildren(master_receive_condition);
  offensive_selector_->AddChildren(search_action_);
  offensive_selector_->AddChildren(patrol_action_);

  //game_start_selctor
  auto obtain_buff_condition_ = std::make_shared<rrts::decision::PreconditionNode>("obtain buff condition", blackboard_ptr_,
                                                                                   offensive_selector_,
                                                                                   [&](){
                                                                                     if (blackboard_ptr_->GetBuffStatus()
                                                                                         == rrts::decision::BuffStatus::SELF) {
                                                                                       return true;
                                                                                     } else {
                                                                                       return false;
                                                                                     }
                                                                                   },
                                                                                   rrts::decision::AbortType::BOTH);
  auto game_start_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_start_selector", blackboard_ptr_);
  game_start_selector_->AddChildren(obtain_buff_condition_);
  game_start_selector_->AddChildren(without_buff_selector_);

  //game_status_selector
  auto game_stop_condition_ = std::make_shared<rrts::decision::PreconditionNode>("game_stop_condition", blackboard_ptr_,
                                                                                 wait_action_,
                                                                                 [&]() {
                                                                                   if (blackboard_ptr_->GetGameProcess()
                                                                                       != rrts::decision::GameProcess::FIGHT) {
                                                                                     return true;
                                                                                   } else {
                                                                                     return false;
                                                                                   }
                                                                                 },
                                                                                 rrts::decision::AbortType::BOTH);

  auto game_status_selector_ = std::make_shared<rrts::decision::SelectorNode>("game_status_selector", blackboard_ptr_);
  game_status_selector_->AddChildren(game_stop_condition_);
  game_status_selector_->AddChildren(game_start_selector_);

  auto wing_stop_condition = std::make_shared<rrts::decision::PreconditionNode>("wing bot stop condition", blackboard_ptr_,
                                                                                wing_auxiliary_action,
                                                                                [&]() {
                                                                                  if (blackboard_ptr_->GetGameProcess()
                                                                                      != rrts::decision::GameProcess::FIGHT) {
                                                                                    return true;
                                                                                  } else {
                                                                                    return false;
                                                                                  }
                                                                                }, rrts::decision::AbortType::BOTH);

  auto wing_dmp_condition_ = std::make_shared<rrts::decision::PreconditionNode>("wing dmp condition",
                                                                                    blackboard_ptr_,
                                                                                    escape_action_,
                                                                                    [&](){
                                                                                      if (blackboard_ptr_->HurtedPerSecond() > 400
                                                                                          || blackboard_ptr_->GetSentBulletStatus()) {
                                                                                        return true;
                                                                                      } else {
                                                                                        return false;
                                                                                      }
                                                                                    },
                                                                                    rrts::decision::AbortType::LOW_PRIORITY);

  auto wing_receive_condition = std::make_shared<rrts::decision::PreconditionNode>("wing receive condition", blackboard_ptr_,
                                                                              auxiliary_action,
                                                                              [&]() {
                                                                                if (blackboard_ptr_->GetAuxiliaryState()) {
                                                                                  return true;
                                                                                } else {
                                                                                  return false;
                                                                                }
                                                                              },
                                                                              rrts::decision::AbortType::LOW_PRIORITY);

  auto wing_detect_condition = std::make_shared<rrts::decision::PreconditionNode>("wing detect condition", blackboard_ptr_,
                                                                                  shoot_action_,
                                                                                  [&]() {
                                                                                    if (blackboard_ptr_->GetEnemyDetected()) {
                                                                                      return true;
                                                                                    } else {
                                                                                      return false;
                                                                                    }
                                                                                  },
                                                                                  rrts::decision::AbortType::BOTH);

  auto wing_color_detect_condition = std::make_shared<rrts::decision::PreconditionNode>("wing color detect condition",
                                                                                        blackboard_ptr_,
                                                                                        color_detected_action_,
                                                                                        [&](){
                                                                                          if (blackboard_ptr_->GetColordetected()
                                                                                              != rrts::decision::ColorDetected ::NONE
                                                                                              && blackboard_ptr_->GetColordetected()
                                                                                                  != rrts::decision::ColorDetected ::FRONT) {
                                                                                            return true;
                                                                                          } else {
                                                                                            return false;
                                                                                          }
                                                                                        },
                                                                                        rrts::decision::AbortType::LOW_PRIORITY);

  auto wing_under_attack_condition = std::make_shared<rrts::decision::PreconditionNode>("wing under attack condition",
                                                                                        blackboard_ptr_, turn_to_hurt_action_,
                                                                                        [&](){
                                                                                          if (blackboard_ptr_->GetArmorAttacked()!=
                                                                                              rrts::decision::ArmorAttacked ::NONE
                                                                                              && blackboard_ptr_->GetArmorAttacked()
                                                                                                  != rrts::decision::ArmorAttacked ::FRONT) {
                                                                                            return true;
                                                                                          } else {
                                                                                            return false;
                                                                                          }
                                                                                        },
                                                                                        rrts::decision::AbortType::LOW_PRIORITY);
  auto wing_bot_selector_ = std::make_shared<rrts::decision::SelectorNode>("wing bot selector", blackboard_ptr_);
  wing_bot_selector_->AddChildren(wing_stop_condition);
  wing_bot_selector_->AddChildren(wing_dmp_condition_);
  wing_bot_selector_->AddChildren(wing_detect_condition);
  wing_bot_selector_->AddChildren(wing_color_detect_condition);
  wing_bot_selector_->AddChildren(wing_under_attack_condition);
  wing_bot_selector_->AddChildren(wing_receive_condition);
  wing_bot_selector_->AddChildren(whirl_action_);

  auto wing_bot_condition = std::make_shared<rrts::decision::PreconditionNode>("wing bot condition", blackboard_ptr_,
                                                                               wing_bot_selector_,
                                                                               [&]() {
                                                                                 if (robot_config.master()) {
                                                                                   return false;
                                                                                 } else {
                                                                                   return true;
                                                                                 }
                                                                               }, rrts::decision::AbortType::NONE);


  auto bot_selector_ = std::make_shared<rrts::decision::SelectorNode>("bot selector", blackboard_ptr_);
  bot_selector_->AddChildren(wing_bot_condition);
  bot_selector_->AddChildren(game_status_selector_);

  rrts::decision::BehaviorTree root(bot_selector_, 25);
  root.Execute();

}


