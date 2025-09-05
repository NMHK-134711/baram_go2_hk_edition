#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"                  // 토픽 구독을 위한 헤더
#include "baram_go2_interfaces/srv/straight.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#define PI 3.14159265358979323846

enum class main_node_mode {
  WAIT       = 0,
  START      = 1,
  TRANSITION = 2,
  PORTHOLE   = 3,
  BEFORETRAFFIC   = 4,
  TRAFFIC    = 5,
  END        = 6
};


