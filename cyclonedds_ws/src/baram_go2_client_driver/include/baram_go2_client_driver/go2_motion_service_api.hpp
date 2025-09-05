/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

#ifndef BARAM__GO2_MOTION_SERVICE_API_HPP_
#define BARAM__GO2_MOTION_SERVICE_API_HPP_

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <future>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "nlohmann/json.hpp"
#include "unitree_api/msg/request.hpp"
#include "unitree_api/msg/response.hpp"


// ==== ROBOT STATE API IDS ==== //
const int32_t API_ID_MS_CHECK_MODE   = 1001;
const int32_t API_ID_MS_SELECT_MODE  = 1002;
const int32_t API_ID_MS_RELEASE_MODE = 1003;
const int32_t API_ID_MS_GET_SILENT   = 1004;
const int32_t API_ID_MS_SET_SILENT   = 1005;

// ==== CLASS ==== //
class Go2MotionSwitcher {
  rclcpp::Node* node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;

  template<typename Request, typename Response>
  nlohmann::json ApiCall(const Request & req) {
    std::promise<typename Response::SharedPtr> res_promise;
    auto res_future = res_promise.get_future();
    const auto api_id = req.header.identity.api_id;

    auto req_sub_ = node_->create_subscription<Response>(
      "/api/motion_switcher/response", 1,
      [&res_promise, api_id](const typename Response::SharedPtr data) {
        if (data->header.identity.api_id == api_id) {
          res_promise.set_value(data);
    }});
    req_pub_->publish(req);

    auto res = res_future.get();
    req_sub_.reset();
    return res;
  }

public:
  explicit Go2MotionSwitcher(rclcpp::Node* node)
    : node_(node),
      req_pub_(node_->create_publisher<unitree_api::msg::Request>(
        "/api/motion_switcher/request", 10)) {}

  // void CheckMode(unitree_api::msg::Request& req) {}
  // void SelectMode(unitree_api::msg::Request& req) {}
  void ReleaseMode(unitree_api::msg::Request& req) {
    if (req.header.identity.api_id != API_ID_MS_RELEASE_MODE) {
      RCLCPP_INFO(node_->get_logger(), "GO2 : \033[1;mRELEASE MODE\033[0m");
    }
    req.header.identity.api_id = API_ID_MS_RELEASE_MODE;
    req_pub_->publish(req);
  }

};


#endif  // BARAM__GO2_MOTION_SERVICE_API_HPP_