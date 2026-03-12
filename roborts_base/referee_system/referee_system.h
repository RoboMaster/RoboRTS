#ifndef ROBORTS_BASE_REFEREE_SYSTEM_H
#define ROBORTS_BASE_REFEREE_SYSTEM_H

#include <cstdint>
#include <memory>
#include "../roborts_sdk/sdk.h"
#include "../ros_dep.h"

namespace roborts_base {

/**
 * @brief ROS API for referee system module.
 *
 * Bridges the RoboMaster referee system protocol with ROS topics.
 * Publishes game status, robot status, heat, damage, and shooting data.
 * Subscribes to projectile supply requests.
 */
class RefereeSystem {
 public:
  /**
   * @brief Constructor of referee system including initialization of sdk and ROS
   * @param handle handler of sdk
   */
  explicit RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle);

  /**
   * @brief Destructor of referee system
   */
  ~RefereeSystem() = default;

  // Non-copyable
  RefereeSystem(const RefereeSystem&) = delete;
  RefereeSystem& operator=(const RefereeSystem&) = delete;

 private:
  /**
   * @brief Initialization of sdk
   */
  void SDK_Init();

  /**
   * @brief Initialization of ROS
   */
  void ROS_Init();

  /** @name Game Related Callbacks */
  ///@{
  void GameStateCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> raw_game_status);
  void GameResultCallback(const std::shared_ptr<roborts_sdk::cmd_game_result> raw_game_result);
  void GameSurvivorCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_survivors> raw_game_survivor);
  ///@}

  /** @name Field Related Callbacks */
  ///@{
  void GameEventCallback(const std::shared_ptr<roborts_sdk::cmd_event_data> raw_game_event);
  void SupplierStatusCallback(const std::shared_ptr<roborts_sdk::cmd_supply_projectile_action> raw_supplier_status);
  ///@}

  /** @name Robot Related Callbacks */
  ///@{
  void RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> raw_robot_status);
  void RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat);
  void RobotBonusCallback(const std::shared_ptr<roborts_sdk::cmd_buff_musk> raw_robot_bonus);
  void RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage);
  void RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot);
  ///@}

  /** @name ROS Callbacks */
  ///@{
  void ProjectileSupplyCallback(const roborts_msgs::ProjectileSupply::ConstPtr projectile_supply);
  ///@}

  //! sdk handler
  std::shared_ptr<roborts_sdk::Handle> handle_;

  //! sdk publisher for projectile supply booking
  std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_supply_projectile_booking>> projectile_supply_pub_;

  //! ros node handler
  ros::NodeHandle ros_nh_;
  //! ros subscriber for projectile supply
  ros::Subscriber ros_sub_projectile_supply_;

  //! Game publishers
  ros::Publisher ros_game_status_pub_;
  ros::Publisher ros_game_result_pub_;
  ros::Publisher ros_game_survival_pub_;

  //! Field publishers
  ros::Publisher ros_bonus_status_pub_;
  ros::Publisher ros_supplier_status_pub_;

  //! Robot publishers
  ros::Publisher ros_robot_status_pub_;
  ros::Publisher ros_robot_heat_pub_;
  ros::Publisher ros_robot_bonus_pub_;
  ros::Publisher ros_robot_damage_pub_;
  ros::Publisher ros_robot_shoot_pub_;

  //! Robot ID from referee system (0xFF = uninitialized)
  uint8_t robot_id_ = 0xFF;
};

} // namespace roborts_base

#endif // ROBORTS_BASE_REFEREE_SYSTEM_H
