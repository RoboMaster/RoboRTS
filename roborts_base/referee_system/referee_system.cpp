#include "referee_system.h"
namespace roborts_base {
RefereeSystem::RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle) :
    handle_(handle) {
  SDK_Init();
  ROS_Init();
}
void RefereeSystem::SDK_Init() {
  handle_->CreateSubscriber<roborts_sdk::cmd_game_state>(REFEREE_GAME_CMD_SET, CMD_GAME_STATUS,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::GameStateCallback,
                                                                   this,
                                                                   std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_game_result>(REFEREE_GAME_CMD_SET, CMD_GAME_RESULT,
                                                          CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                          std::bind(&RefereeSystem::GameResultCallback,
                                                                    this,
                                                                    std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_survivors>(REFEREE_GAME_CMD_SET, CMD_GAME_SURVIVAL,
                                                                   CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                                   std::bind(&RefereeSystem::GameSurvivorCallback,
                                                                             this,
                                                                             std::placeholders::_1));


  handle_->CreateSubscriber<roborts_sdk::cmd_event_data>(REFEREE_BATTLEFIELD_CMD_SET, CMD_BATTLEFIELD_EVENT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::GameEventCallback,
                                                                   this,
                                                                   std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_supply_projectile_action>(REFEREE_BATTLEFIELD_CMD_SET, CMD_SUPPLIER_ACTION,
                                                                       CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                                       std::bind(&RefereeSystem::SupplierStatusCallback,
                                                                                 this,
                                                                                 std::placeholders::_1));


  handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_state>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_STATUS,
                                                               CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                               std::bind(&RefereeSystem::RobotStatusCallback,
                                                                         this,
                                                                         std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_power_heat_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_POWER_HEAT,
                                                              CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                              std::bind(&RefereeSystem::RobotHeatCallback,
                                                                        this,
                                                                        std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_buff_musk>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_BUFF,
                                                        CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                        std::bind(&RefereeSystem::RobotBonusCallback,
                                                                  this,
                                                                  std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_robot_hurt>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_HURT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::RobotDamageCallback,
                                                                   this,
                                                                   std::placeholders::_1));
  handle_->CreateSubscriber<roborts_sdk::cmd_shoot_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_SHOOT,
                                                         CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                         std::bind(&RefereeSystem::RobotShootCallback,
                                                                   this,
                                                                   std::placeholders::_1));



  projectile_supply_pub_ =
      handle_->CreatePublisher<roborts_sdk::cmd_supply_projectile_booking>(REFEREE_SEND_CMD_SET, CMD_REFEREE_SEND_DATA,
                                                                           MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);



}
void RefereeSystem::ROS_Init() {
  //ros publisher
  ros_game_status_pub_ = ros_nh_.advertise<roborts_msgs::GameStatus>("game_status", 30);
  ros_game_result_pub_ = ros_nh_.advertise<roborts_msgs::GameResult>("game_result", 30);
  ros_game_survival_pub_ = ros_nh_.advertise<roborts_msgs::GameSurvivor>("game_survivor", 30);

  ros_bonus_status_pub_ = ros_nh_.advertise<roborts_msgs::BonusStatus>("field_bonus_status", 30);
  ros_supplier_status_pub_ = ros_nh_.advertise<roborts_msgs::SupplierStatus>("field_supplier_status", 30);

  ros_robot_status_pub_ = ros_nh_.advertise<roborts_msgs::RobotStatus>("robot_status", 30);
  ros_robot_heat_pub_ = ros_nh_.advertise<roborts_msgs::RobotHeat>("robot_heat", 30);
  ros_robot_bonus_pub_ = ros_nh_.advertise<roborts_msgs::RobotBonus>("robot_bonus", 30);
  ros_robot_damage_pub_ = ros_nh_.advertise<roborts_msgs::RobotDamage>("robot_damage", 30);
  ros_robot_shoot_pub_ = ros_nh_.advertise<roborts_msgs::RobotShoot>("robot_shoot", 30);

  //ros subscriber
  ros_sub_projectile_supply_ = ros_nh_.subscribe("projectile_supply", 1, &RefereeSystem::ProjectileSupplyCallback, this);

}

void RefereeSystem::GameStateCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> raw_game_status){
  roborts_msgs::GameStatus game_status;
  game_status.game_status = raw_game_status->game_progress;
  game_status.remaining_time = raw_game_status->stage_remain_time;
  ros_game_status_pub_.publish(game_status);
}

void RefereeSystem::GameResultCallback(const std::shared_ptr<roborts_sdk::cmd_game_result> raw_game_result){
  roborts_msgs::GameResult game_result;
  game_result.result = raw_game_result->winner;
  ros_game_result_pub_.publish(game_result);
}

void RefereeSystem::GameSurvivorCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_survivors> raw_game_survivor){
  roborts_msgs::GameSurvivor game_survivor;
  game_survivor.red3 = raw_game_survivor->robot_legion>>2&1;
  game_survivor.red4 = raw_game_survivor->robot_legion>>3&1;
  game_survivor.blue3 = raw_game_survivor->robot_legion>>10&1;
  game_survivor.blue4 = raw_game_survivor->robot_legion>>11&1;
  ros_game_survival_pub_.publish(game_survivor);
}

void RefereeSystem::GameEventCallback(const std::shared_ptr<roborts_sdk::cmd_event_data> raw_game_event){
  roborts_msgs::BonusStatus bonus_status;
  bonus_status.red_bonus = raw_game_event->event_type>>12&3;
  bonus_status.blue_bonus = raw_game_event->event_type>>14&3;
  ros_bonus_status_pub_.publish(bonus_status);
}

void RefereeSystem::SupplierStatusCallback(const std::shared_ptr<roborts_sdk::cmd_supply_projectile_action> raw_supplier_status){
  roborts_msgs::SupplierStatus supplier_status;
  supplier_status.status = raw_supplier_status->supply_projectile_step;
  ros_supplier_status_pub_.publish(supplier_status);
}

void RefereeSystem::RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> raw_robot_status){
  roborts_msgs::RobotStatus robot_status;

  if(robot_id_ != raw_robot_status->robot_id){

    switch (raw_robot_status->robot_id){
      case 3:
        robot_status.id = 3;
        robot_id_ = 3;
        break;
      case 4:
        robot_status.id = 4;
        robot_id_ = 4;
        break;
      case 13:
        robot_status.id = 13;
        robot_id_ = 13;
        break;
      case 14:
        robot_status.id = 14;
        robot_id_ = 14;
        break;
      default:
        robot_status.id = raw_robot_status->robot_id;
        LOG_ERROR<<"For AI challenge, please set robot id to Blue3/4 or Red3/4 in the referee system main control module";
        return;
    }
  }
  else{
    robot_status.id = raw_robot_status->robot_id;
  }

  robot_status.level = raw_robot_status->robot_level;
  robot_status.remain_hp = raw_robot_status->remain_HP;
  robot_status.max_hp = raw_robot_status->max_HP;
  robot_status.heat_cooling_limit = raw_robot_status->shooter_heat0_cooling_limit;
  robot_status.heat_cooling_rate = raw_robot_status->shooter_heat0_cooling_rate;
  robot_status.chassis_output = raw_robot_status->mains_power_chassis_output;
  robot_status.gimbal_output = raw_robot_status->mains_power_gimbal_output;
  robot_status.shooter_output = raw_robot_status->mains_power_shooter_output;
  ros_robot_status_pub_.publish(robot_status);
}

void RefereeSystem::RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat){
  roborts_msgs::RobotHeat robot_heat;
  robot_heat.chassis_volt = raw_robot_heat->chassis_volt;
  robot_heat.chassis_current = raw_robot_heat->chassis_current;
  robot_heat.chassis_power = raw_robot_heat->chassis_power;
  robot_heat.chassis_power_buffer = raw_robot_heat->chassis_power_buffer;
  robot_heat.shooter_heat = raw_robot_heat->shooter_heat0;
  ros_robot_heat_pub_.publish(robot_heat);
}

void RefereeSystem::RobotBonusCallback(const std::shared_ptr<roborts_sdk::cmd_buff_musk> raw_robot_bonus){
  roborts_msgs::RobotBonus robot_bonus;
  robot_bonus.bonus = raw_robot_bonus->power_rune_buff>>2&1;
  ros_robot_bonus_pub_.publish(robot_bonus);
}

void RefereeSystem::RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage){
  roborts_msgs::RobotDamage robot_damage;
  robot_damage.damage_type = raw_robot_damage->hurt_type;
  robot_damage.damage_source = raw_robot_damage->armor_id;
  ros_robot_damage_pub_.publish(robot_damage);
}

void RefereeSystem::RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot){
  roborts_msgs::RobotShoot robot_shoot;
  robot_shoot.frequency = raw_robot_shoot->bullet_freq;
  robot_shoot.speed = raw_robot_shoot->bullet_speed;
  ros_robot_shoot_pub_.publish(robot_shoot);
}

void RefereeSystem::ProjectileSupplyCallback(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply){
  if (robot_id_ == 0xFF) {
    ROS_ERROR("Can not get robot id before requesting for projectile supply.");
    return;
  }
  roborts_sdk::cmd_supply_projectile_booking raw_projectile_booking;
  raw_projectile_booking.supply_projectile_id = 1;
  raw_projectile_booking.supply_robot_id = robot_id_;
  raw_projectile_booking.supply_num = projectile_supply->number/50*50;
  projectile_supply_pub_->Publish(raw_projectile_booking);
}

}
