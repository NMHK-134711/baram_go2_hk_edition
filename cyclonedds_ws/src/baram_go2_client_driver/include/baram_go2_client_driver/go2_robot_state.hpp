/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 *      https://github.com/Unitree-Go2-Robot/go2_driver
 */


#ifndef BARAM__GO2_ROBOT_STATE_HPP_
#define BARAM__GO2_ROBOT_STATE_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "unitree_go/msg/detail/sport_mode_state__struct.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

#include "go2_robot_state_api.hpp"


#define TOPIC_HIGHSTATE "lf/sportmodestate"
#define TOPIC_API_REQ   "api/sport/request"
#define TOPIC_LOWSTATE  "lowstate"


class Go2RobotState : public rclcpp::Node {

  /**
   * @brief Unitree Api client. Using this instance for call APIs(Motion Switcher).
   */
  Go2RobotStateApi robot_state_client_;

  // === PUBLISHERS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr go2_imu_pub_;

  // === SUBSCRIBERS
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmode_state_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;

  // === MSGS
  unitree_go::msg::SportModeState state_;
  unitree_go::msg::LowState low_state_;
  sensor_msgs::msg::Imu go2_imu_;

  double px0_{}, py0_{}, yaw0_{};
  double imu_x0_, imu_y0_, imu_z0_, imu_w0_;
  double imu_x_, imu_y_, imu_z_, imu_w_;

  bool low_loop_active_ = false;

  std::thread t1_;
  std::mutex mtx_;

  /**
   * @brief Atomic variable for High/Low control configurations.
   * The modes are defined in ModeSetFlag. More informations are defined in 'include/baram_go2_client_driver/go2_api.hpp'
   */
  int32_t status_ = 0;

public:
  explicit Go2RobotState();

  /**
   * @brief Store the current pose of the robot.
   */
  void SetInitState();

  /**
   * @brief Load the stored pose of the robot.
   */
  void GetInitState();

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg);
  void LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);

};


#endif  // BARAM__GO2_ROBOT_STATE_HPP_