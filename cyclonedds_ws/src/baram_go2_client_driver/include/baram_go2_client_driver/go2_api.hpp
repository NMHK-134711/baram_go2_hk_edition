/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 *      https://github.com/Unitree-Go2-Robot/go2_driver
 */


#ifndef BARAM__GO2_API_HPP_
#define BARAM__GO2_API_HPP_

#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"


// ==== SPORT MODE API IDS === //
const int32_t API_ID_DAMP           = 1001;
const int32_t API_ID_BALANCESTAND   = 1002;
const int32_t API_ID_STOPMOVE       = 1003;
const int32_t API_ID_STANDUP        = 1004;
const int32_t API_ID_STANDDOWN      = 1005;
const int32_t API_ID_RECOVERYSTAND  = 1006;
const int32_t API_ID_EULER          = 1007;
const int32_t API_ID_MOVE           = 1008;
const int32_t API_ID_SIT            = 1009;
const int32_t API_ID_RISESIT        = 1010;

// Currently unavaliable apis ...
// const int32_t API_ID_BODYHEIGHT     = 1013;
// And so on.

const int32_t API_ID_SPEEDLEVEL     = 1015;
const int32_t API_ID_HELLO          = 1016;
const int32_t API_ID_STRETCH        = 1017;

const int32_t API_ID_CONTENT        = 1020;
const int32_t API_ID_DANCE1         = 1022;
const int32_t API_ID_DANCE2         = 1023;

const int32_t API_ID_SWITCHJOYSTICK = 1027;
const int32_t API_ID_POSE           = 1028;
const int32_t API_ID_SCRAPE         = 1029;

const int32_t API_ID_FRONTFLIP      = 1030;
const int32_t API_ID_FRONTJUMP      = 1031;
const int32_t API_ID_FRONTPOUNCE    = 1032;

const int32_t API_ID_HEART          = 1036;

const int32_t API_ID_STATICWALK     = 1061;
const int32_t API_ID_TROTRUN        = 1062;
const int32_t API_ID_ECONOMICGAIT   = 1063;

const int32_t API_ID_LEFTFLIP       = 2041;
const int32_t API_ID_BACKFLIP       = 2043;
const int32_t API_ID_HANDSTAND      = 2044;
const int32_t API_ID_FREEWALK       = 2045;
const int32_t API_ID_FREEBOUND      = 2046;
const int32_t API_ID_FREEJUMP       = 2047;
const int32_t API_ID_FREEAVOID      = 2048;
const int32_t API_ID_CLASSICWALK    = 2049;

const int32_t API_ID_WALKUPRIGHT      = 2050;
const int32_t API_ID_CROSSSTEP        = 2051;
const int32_t API_ID_AUTORECOVERY_SET = 2054;
const int32_t API_ID_AUTORECOVERY_GET = 2055;
const int32_t API_ID_SWITCHAVOIDMODE  = 2058;
// ============================= //

#pragma pack(1)
struct PathPoint {
  float timeFromStart;
  float x;
  float y;
  float yaw;
  float vx;
  float vy;
  float vyaw;
};
#pragma pack()


class Go2Api {
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr res_sub_;
  rclcpp::Node *node_;

  bool log_;

public:
  explicit Go2Api(rclcpp::Node *node);

  template<typename Request, typename Response>
  typename Response::SharedPtr ApiCall(const Request& req, std::chrono::milliseconds timeout) {
    std::promise<typename Response::SharedPtr> res_promise;
    auto res_future = res_promise.get_future();
    auto api_id = req.header.identity.api_id;
    auto sub = node_->create_subscription<Response>(
      "/api/sport/response", 1,
      [&res_promise, api_id](const typename Response::SharedPtr resp) {
        if (resp->header.identity.api_id == api_id)
          res_promise.set_value(resp);
    });

    req_pub_->publish(req);

    auto responce = res_future.get();
    sub.reset();
    return responce;
  }

  // Api Srv Functions
  void Damp(unitree_api::msg::Request& req);
  void BalanceStand(unitree_api::msg::Request& req);
  void StopMove(unitree_api::msg::Request& req);
  void StandUp(unitree_api::msg::Request& req);
  void StandDown(unitree_api::msg::Request& req);
  void RecoveryStand(unitree_api::msg::Request& req);
  void Euler(
    unitree_api::msg::Request& req,
    float roll, float pitch, float yaw);
  void Move(
    unitree_api::msg::Request& req,
    float vx, float vy, float vyaw);
  void Sit(unitree_api::msg::Request& req);
  void RiseSit(unitree_api::msg::Request& req);
  // void BodyHeight(unitree_api::msg::Request& req);
  void SpeedLevel(unitree_api::msg::Request& req, int level);
  void Hello(unitree_api::msg::Request& req);
  void Stretch(unitree_api::msg::Request& req);
  void SwitchJoysitck(unitree_api::msg::Request& req, bool flag);
  void Content(unitree_api::msg::Request& req);
  void Pose(unitree_api::msg::Request& req, bool flag);
  void Scrape(unitree_api::msg::Request& req);
  void FrontFlip(unitree_api::msg::Request& req);
  void FrontJump(unitree_api::msg::Request& req);
  void Dance1(unitree_api::msg::Request& req);
  void Dance2(unitree_api::msg::Request& req);
  void Heart(unitree_api::msg::Request& req);
  void StaticWalk(unitree_api::msg::Request& req);
  void TrotRun(unitree_api::msg::Request& req);
  void EconomicGait(unitree_api::msg::Request& req);
  void LeftFlip(unitree_api::msg::Request& req);
  void BackFilp(unitree_api::msg::Request& req);
  void HandStand(unitree_api::msg::Request& req, bool flag);
  void FreeWalk(unitree_api::msg::Request& req, bool flag);
  void FreeBound(unitree_api::msg::Request& req, bool flag);
  void FreeJump(unitree_api::msg::Request& req, bool flag);
  void FreeAvoid(unitree_api::msg::Request& req, bool flag);
  void ClassicWalk(unitree_api::msg::Request& req, bool flag);
  void WalkUpgright(unitree_api::msg::Request& req, bool flag);
  // NOTICE: AutoRecoveryGet is only avaliable on ROS2 Humble
  void AutoRecoveryGet(unitree_api::msg::Request& req, bool flag);
  void SwitchAvoidMode(unitree_api::msg::Request& req);
};

#endif  // BARAM__GO2_API_HPP_