/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

#ifndef BARAM__GO2_SLAM_API_HPP_
#define BARAM__GO2_SLAM_API_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <utility>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"


// ==== UNITREE SLAM API IDS ==== //
const int32_t API_ID_STOP_NODE           = 1901;
const int32_t API_ID_START_MAPPING_PL    = 1801;
const int32_t API_ID_END_MAPPING_PL      = 1802;
const int32_t API_ID_START_RELOCATION_PL = 1804;
const int32_t API_ID_POSE_NAV_PL         = 1102;
const int32_t API_ID_PAUSE_NAV           = 1201;
const int32_t API_ID_RESUME_NAV          = 1202;

// ==== Data Structures ==== //
class poseData {
public:
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float q_x = 0.0f;
  float q_y = 0.0f;
  float q_z = 0.0f;
  float q_w = 1.0f;
  int mode = 1;
  float speed = 0.8f;
  std::string toJsonStr() const {
    nlohmann::json j;
    j["data"]["targetPose"]["x"] = x;
    j["data"]["targetPose"]["y"] = y;
    j["data"]["targetPose"]["z"] = z;
    j["data"]["targetPose"]["q_x"] = q_x;
    j["data"]["targetPose"]["q_y"] = q_y;
    j["data"]["targetPose"]["q_z"] = q_z;
    j["data"]["targetPose"]["q_w"] = q_w;
    j["data"]["mode"] = mode;
    j["data"]["speed"] = speed;
    return j.dump(4);
  }

  void printInfo() const {
    std::cout <<
      "x:" << x << " y:" << y << " z:" << z << " q_x:" <<
      q_x << " q_y:" << q_y << " q_z:" << q_z << " q_w:" << q_w << std::endl;
  }
};

// struct Responses {
//   std::string name;
//   int32_t status;
// };

// void from_json(const nlohmann::json& j, Responses& item) {
//   j.at("name").get_to(item.name);
//   j.at("status").get_to(item.status);
// }


// ==== CLASS ==== //
class Go2SlamApi {
  rclcpp::Node* node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;
  rclcpp::Subscription<unitree_api::msg::Response>::SharedPtr resp_sub_;  //

  template<typename Request, typename Response>
  nlohmann::json ApiCall(const Request & req) {
    std::promise<typename Response::SharedPtr> res_promise;
    auto res_future = res_promise.get_future();
    const auto api_id = req.header.identity.api_id;

    auto req_sub_ = node_->create_subscription<Response>(
      "/api/slam_operate/response", 1,
      [&res_promise, api_id](const typename Response::SharedPtr data) {
        if (data->header.identity.api_id == api_id)
          res_promise.set_value(data);
    });
    req_pub_->publish(req);

    auto res = res_future.get();
    req_sub_.reset();
    return res;
  }

public:
  explicit Go2SlamApi(rclcpp::Node* node)
    : node_(node),
      req_pub_(node_->create_publisher<unitree_api::msg::Request>(
        "/api/slam_operate/request", 10))
  {
    resp_sub_ = node_->create_subscription<unitree_api::msg::Response>(
      "/api/slam_operate/response",
      1,
      [this](const unitree_api::msg::Response::SharedPtr msg) {
        RCLCPP_DEBUG(node_->get_logger(),
            "SLAM response api_id=%ld error_code=%d",
            msg->header.identity.api_id,
            msg->header.status.code);
      });
  }

  void StopNode(unitree_api::msg::Request& req) {
    if (req.header.identity.api_id != API_ID_STOP_NODE)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mSTOP NODE\033[0m");
    req.header.identity.api_id = API_ID_STOP_NODE;
    nlohmann::json js;
    js["data"] = nlohmann::json::object();
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

  void StartMapping(unitree_api::msg::Request& req) {
    if (req.header.identity.api_id != API_ID_START_MAPPING_PL)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mSTART MAPPING\033[0m");
    req.header.identity.api_id = API_ID_START_MAPPING_PL;
    nlohmann::json js;
    js["data"]["slam_type"] = "indoor";
    req.parameter = js.dump();

    req_pub_->publish(req);
    RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;35mrequest pub\033[0m");
  }

  void EndMapping(unitree_api::msg::Request& req, std::string& address) {
    if (req.header.identity.api_id != API_ID_END_MAPPING_PL)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mEND MAPPING\033[0m");
    RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: SAVE DIR is %s.", address.c_str());
    req.header.identity.api_id = API_ID_END_MAPPING_PL;
    nlohmann::json js;
    js["data"]["address"] = address;
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

  void ReLocation(
    unitree_api::msg::Request& req,
    const poseData& init_pose,
    const std::string& map_address
  ) {
    if (req.header.identity.api_id != API_ID_START_RELOCATION_PL)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mSTART RELOCATION\033[0m");
    req.header.identity.api_id = API_ID_START_RELOCATION_PL;
    nlohmann::json js;
    js["data"]["x"]   = init_pose.x;
    js["data"]["y"]   = init_pose.y;
    js["data"]["z"]   = init_pose.z;
    js["data"]["q_x"] = init_pose.q_x;
    js["data"]["q_y"] = init_pose.q_y;
    js["data"]["q_z"] = init_pose.q_z;
    js["data"]["q_w"] = init_pose.q_w;
    js["data"]["address"] = map_address.c_str();
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

  void PoseNav(
    unitree_api::msg::Request& req,
    const poseData& target_pose,
    int mode = 1,
    float speed = 0.8f
  ) {
    if (req.header.identity.api_id != API_ID_POSE_NAV_PL)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: POSE NAV");
    req.header.identity.api_id = API_ID_POSE_NAV_PL;
    nlohmann::json js;
    js["data"]["targetPose"]["x"] = target_pose.x;
    js["data"]["targetPose"]["y"] = target_pose.y;
    js["data"]["targetPose"]["z"] = target_pose.z;
    js["data"]["targetPose"]["q_x"] = target_pose.q_x;
    js["data"]["targetPose"]["q_y"] = target_pose.q_y;
    js["data"]["targetPose"]["q_z"] = target_pose.q_z;
    js["data"]["targetPose"]["q_w"] = target_pose.q_w;
    js["data"]["mode"]  = mode;
    js["data"]["speed"] = speed;
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

  void PauseNav(unitree_api::msg::Request& req) {
    if (req.header.identity.api_id != API_ID_PAUSE_NAV)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mPAUSE NAV\033[0m");
    req.header.identity.api_id = API_ID_PAUSE_NAV;
    nlohmann::json js;
    js["data"] = nlohmann::json::object();
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

  void ResumeNav(unitree_api::msg::Request& req) {
    if (req.header.identity.api_id != API_ID_RESUME_NAV)
      RCLCPP_INFO(node_->get_logger(), "GO2 SLAM: \033[1;32mRESUME NAV\033[0m");
    req.header.identity.api_id = API_ID_RESUME_NAV;
    nlohmann::json js;
    js["data"] = nlohmann::json::object();
    req.parameter = js.dump();
    req_pub_->publish(req);
  }

};


#endif  // BARAM__GO2_SLAM_API_HPP_