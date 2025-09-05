/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 *      https://github.com/Unitree-Go2-Robot/go2_driver
 */


#ifndef BARAM__GO2_CLIENT_DRIVER_HPP_
#define BARAM__GO2_CLIENT_DRIVER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "unitree_go/msg/detail/sport_mode_state__struct.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "unitree_go/msg/low_state.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

#include "go2_api.hpp"
#include "go2_robot_state_api.hpp"
#include "go2_motion_service_api.hpp"
#include "go2_slam_api.hpp"

#include "motor_crc.h"

#define TOPIC_HIGHSTATE "lf/sportmodestate"
#define TOPIC_LOWCMD    "lowcmd"
#define TOPIC_API_REQ   "api/sport/request"
#define TOPIC_LOWSTATE  "lowstate"

#define MAP_STORE_FILE_DIR_NAME "/home/go/unitree_ros2/map.pcd"

// constexpr int HIGHLEVEL = 0xEE;
// constexpr int LOWLEVEL = 0xFF;
// constexpr int TRIGERLEVEL = 0xF0;
// constexpr double PosStopF = (2.146E+9f);
// constexpr double VelStopF = (16000.0f);

enum ModeSetFlag {
  // NORMAL_STAND,
  STAND_UP,
  BALANCE_STAND,
  STAND_DOWN,
  DAMP,
  STOP_MOVE,
  NAVIGATION,
  CRAWLING,
  JUMP_FORWARD,
  RUNNING
};

enum class CrawlState {
  WAIT       = 0,
  ALIGN      = 1,
  TRANSITION = 2,
  TROT       = 3,
  DONE       = 4
};

enum class SlamMode {
  START_MAPPING = 0,
  END_MAPPING   = 1,
  RELOCATION    = 2,
  PAUSE_NAV     = 3,
  RESEME_NAV    = 4
};


class Go2HighControl : public rclcpp::Node {
  /**
   * @brief Unitree Api client. Using this instance for call APIs(sport mode [Motion Services]).
   */
  Go2Api api_client_;
  Go2SlamApi api_slam_;

  /**
   * @brief Unitree Api client. Using this instance for call APIs(Motion Switcher).
   */
  Go2RobotStateApi robot_state_client_;
  Go2MotionSwitcher motion_switcher_client_;

  // === PUBLISHERS
  rclcpp::Publisher<unitree_go::msg::LowCmd>::SharedPtr low_cmd_pub_;

  // === SUBSCRIBERS
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sportmode_state_sub_;
  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr low_state_sub_;
  rclcpp::Subscription<unitree_api::msg::Request>::SharedPtr api_req_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // === TIMERS
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr low_loop_timer_;
  std::chrono::steady_clock::time_point last_act_timer_;

  // === MSGS
  unitree_go::msg::SportModeState state_;
  geometry_msgs::msg::Twist cmd_vel_;
  unitree_api::msg::Request api_req_;
  unitree_go::msg::LowCmd low_cmd_;
  unitree_go::msg::LowState low_state_;

  // === CRAWL
  CrawlState crawl_state_;
  std::vector<double> crawl_target_pos_;
  std::array<double, 12> crawl_start_pos_{};
  std::array<std::vector<double>, 12> joint_control_pts_;
  int duration_align_ = 1000;
  int transition_duration_ = 400;
  int transition_step_ = 0;
  int trot_duration_ = 300;
  int trot_step_ = 0;
  int trot_cycle_cnt_ = 0;
  double crawl_align_percent_ = 0.0;
  std::atomic<bool> crawl_{false};

  // === JUMP
  bool waiting_for_jump_ = true;

  // === PD Control
  double Kp_ = 60.0, Kd_ = 5.0;

  double px0_{}, py0_{}, yaw0_{};
  double ct_{};
  float dt_ = 0.001;

  bool low_loop_active_ = false;

  std::thread t1_;
  std::mutex mtx_;

  /**
   * @brief Atomic variable for High/Low control configurations.
   * The modes are defined in ModeSetFlag. More informations are defined in 'include/baram_go2_client_driver/go2_api.hpp'
   */
  std::atomic<ModeSetFlag> mode_;

  int32_t status_ = 0;

public:
  explicit Go2HighControl();

  /**
   * @brief Low Command Control Switcher. It use to configure the motor status
   */
  void SwitchLowCmd();
  /**
   * @brief High Command Control Switcher. It use to change the sport mode (Call APIs)
   */
  void SwitchHighCmd();

  /**
   * @brief Store the current pose of the robot.
   */
  void SetInitState();

  /**
   * @brief Load the stored pose of the robot.
   */
  void GetInitState();

  void InitCrawlPos();

  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg);
  void LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg);
  void ApiReqHandler(const unitree_api::msg::Request::SharedPtr msg);
  void CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Handle the control of the Unitree Go. Try to change modes to store/load 'mode_'.
   */
  void ControlHandler();

  /**
   * @brief Handle the control for Crawling. It can be used to template as the control the motor in low level.
   */
  void LowCmdWrite();

  /**
   * @brief Cubic Spline Method. For CRAWLING. It can be used to template as the control the motor in low level.
   */
  void TrotSpline();
  double EvalCatmullRomClosed(const std::vector<double>& p, double t01) const;

  /**
   * @brief Main function of CRAWL Motion control.
   */
  void Crawl();
  void StopCrawl();

};


#endif  // BARAM__GO2_CLIENT_DRIVER_HPP_