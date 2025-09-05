/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

#ifndef BARAM__GO2_ROBOT_STATE_API_HPP_
#define BARAM__GO2_ROBOT_STATE_API_HPP_

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
const int32_t API_ID_SERVICE_SWITCH          = 1001;
const int32_t API_ID_SERVICE_SET_REPORT_FREQ = 1002;
const int32_t API_ID_SERVICE_LIST            = 1003;

// ==== Data Structures ==== //
struct ServiceState {
  std::string name;
  int32_t status;
  int32_t protect;
};

struct ServiceSwitchRequest {
  std::string name;
  int32_t swit;
};

struct ServiceSwitchResponse {
  std::string name;
  int32_t status;
};

struct SetReportFreqRequest {
  int32_t interval;
  int32_t duration;
};

// ==== JSON Serialization & Deserialization ==== //
void from_json(const nlohmann::json& j, ServiceState& item) {
  j.at("name").get_to(item.name);
  j.at("status").get_to(item.status);
  j.at("protect").get_to(item.protect);
}

void to_json(nlohmann::json& j, ServiceSwitchRequest& item) {
  j = nlohmann::json{{"name", item.name}, {"switch", item.swit}};
}

void from_json(const nlohmann::json& j, ServiceSwitchResponse& item) {
  j.at("name").get_to(item.name);
  j.at("status").get_to(item.status);
}

void to_json(nlohmann::json& j, SetReportFreqRequest& item) {
  j = nlohmann::json{{"interval", item.interval}, {"duration", item.duration}};
}

// ==== CLASS ==== //
class Go2RobotStateApi {
  rclcpp::Node* node_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_pub_;

  template<typename Request, typename Response>
  nlohmann::json ApiCall(const Request & req) {
    std::promise<typename Response::SharedPtr> res_promise;
    auto res_future = res_promise.get_future();
    const auto api_id = req.header.identity.api_id;

    auto req_sub_ = node_->create_subscription<Response>(
      "/api/robot_state/response", rclcpp::QoS(1),
      [&res_promise, api_id](const typename Response::SharedPtr data) {
        if (data->header.identity.api_id == api_id) {
          res_promise.set_value(data);
        }
      });
    req_pub_->publish(req);

    auto res = *res_future.get();
    return nlohmann::json::parse(res.data.data());
  }

  uint64_t GetSystemUptimeInNanoseconds() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
  }

public:
  explicit Go2RobotStateApi(rclcpp::Node* node)
    : node_(node),
      req_pub_(node_->create_publisher<unitree_api::msg::Request>(
        "/api/robot_state/request", 10)) {}

  int32_t ServiceList(std::vector<ServiceState>& list) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = API_ID_SERVICE_LIST;
    req.header.identity.id = GetSystemUptimeInNanoseconds();
    auto js = ApiCall<unitree_api::msg::Request, unitree_api::msg::Response>(req);
    js.get_to(list);
    return 0;
  }

  int32_t ServiceSwitch(const std::string& name, int32_t swit, int32_t& status) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = API_ID_SERVICE_SWITCH;
    req.header.identity.id = GetSystemUptimeInNanoseconds();
    nlohmann::json js;
    js["name"] = name;
    js["switch"] = swit;
    req.parameter = js.dump();

    auto js_res = ApiCall<unitree_api::msg::Request, unitree_api::msg::Response>(req);
    ServiceSwitchResponse res;
    js_res.get_to(res);
    status = res.status;
    return 0;
  }

  int32_t SetReportFreq(int32_t interval, int32_t duration) {
    unitree_api::msg::Request req;
    req.header.identity.api_id = API_ID_SERVICE_SET_REPORT_FREQ;
    nlohmann::json js;
    js["interval"] = interval;
    js["duration"] = duration;
    req.parameter = js.dump();
    req.header.identity.id = GetSystemUptimeInNanoseconds();

    req_pub_->publish(req);
    return 0;
  }

};


#endif  // BARAM__GO2_ROBOT_STATE_API_HPP_