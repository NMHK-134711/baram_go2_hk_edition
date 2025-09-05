/**
 * Author: Kho Geonwoo | Jang Seungwon | Lee Jinwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

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

//#include "go2_robot_state_api.hpp"


#define TOPIC_HIGHSTATE "lf/sportmodestate"
#define TOPIC_API_REQ   "api/sport/request"
#define TOPIC_LOWSTATE  "lowstate"


#include "baram_go2_client_driver/go2_robot_state.hpp"

// #define DEBUG_POS_
// #define DEBUG_IMU_
#define DEBUG_IMU_QT_

// #define DEBUG_T1_

Go2RobotState::Go2RobotState()
 : Node("go2_robot_state"),
   robot_state_client_(this) {
  RCLCPP_INFO(this->get_logger(), "\033[1;34mGO2 ROBOT STATE INIT\033[0m");

  sportmode_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
    TOPIC_HIGHSTATE, 1, [this](const unitree_go::msg::SportModeState::SharedPtr data) {
      HighStateHandler(data);
  });

  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
    TOPIC_LOWSTATE, 10, [this](const unitree_go::msg::LowState::SharedPtr data) {
      LowStateHandler(data);
  });

  go2_imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("go2_imu", 50);

  SetInitState();

#ifdef DEBUG_T1_
  t1_ = std::thread([this] {
    RCLCPP_INFO(this->get_logger(), "\033[1;35mt1 thread\033[0m");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  });
#endif // DEBUG_T1_

}

void Go2RobotState::SetInitState() {
  px0_ = state_.position[0];
  py0_ = state_.position[1];
  yaw0_ = state_.imu_state.rpy[2];

  imu_x0_ = state_.imu_state.quaternion[0];
  imu_y0_ = state_.imu_state.quaternion[1];
  imu_z0_ = state_.imu_state.quaternion[2];
  imu_w0_ = state_.imu_state.quaternion[3];

  RCLCPP_INFO(
    this->get_logger(), "\n\033[1;32mSet initial position:\n\tx0: %f\n\ty0: %f\n\tyaw0: %f\033[0m",
    px0_, py0_,yaw0_
  );
  RCLCPP_INFO(
    this->get_logger(), "\n\033[1;32mSet initial position:\n\tx: %f\n\ty: %f\n\tz: %f\n\tw: %f\033[0m",
    imu_x0_, imu_y0_,imu_z0_,imu_w0_
  );
}

void Go2RobotState::GetInitState() {
  RCLCPP_INFO(
    this->get_logger(), "\n\033[1;32mGet initial position:\n\tx0: %f\n\ty0: %f\n\tyaw0: %f\033[0m",
    px0_, py0_,yaw0_
  );
}

void Go2RobotState::HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
  state_ = *msg;

  #ifdef DEBUG_POS_
    RCLCPP_INFO(
      this->get_logger(), "Position:\033[1;34m\n\t%f\n\t%f\n\t%f",
      state_.position[0], state_.position[1], state_.position[2]
    );

    RCLCPP_INFO(
      this->get_logger(), "IMU rpy:\033[1;34m\n\t%f\n\t%f\n\t%f",
      state_.imu_state.rpy[0], state_.imu_state.rpy[1], state_.imu_state.rpy[2]
    );
  #endif  // DEBUG_POS_

  #ifdef DEBUG_IMU_
    RCLCPP_INFO(this->get_logger(),
      "Current positions\nPosition:\t%f\t%f\t%f\n\tIMU:\t%f\t%f\t%f",
      state_.position[0], state_.position[1], state_.position[2],
      state_.imu_state.rpy[0], state_.imu_state.rpy[1], state_.imu_state.rpy[2]);
  #endif

  #ifdef DEBUG_IMU_QT_
    RCLCPP_INFO(this->get_logger(),
      "\nCurrent positions\n\tPosition:\n\t%f\n\t%f\n\t%f\n\tQt:\n\t%f\n\t%f\n\t%f\n\t%f",
      state_.position[0], state_.position[1], state_.position[2],
      state_.imu_state.quaternion[0], state_.imu_state.quaternion[1],
      state_.imu_state.quaternion[2], state_.imu_state.quaternion[3]);
    RCLCPP_INFO(this->get_logger(),
      "\n\tGyro:\n\t%f\n\t%f\n\t%f\n\tLinear acc.:\n\t%f\n\t%f\n\t%f",
      state_.imu_state.gyroscope[0],state_.imu_state.gyroscope[1],state_.imu_state.gyroscope[2],
      state_.imu_state.accelerometer[0], state_.imu_state.accelerometer[1],state_.imu_state.accelerometer[2]);
  #endif

}

void Go2RobotState::LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Go2RobotState>();
  node->GetInitState();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}