/**
 * Author: Kho Geonwoo
 *
 * REFERENCE: unitre_ros2
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

#include <cstdio>
#include <string>

#include "nlohmann/json.hpp"
#include "baram_go2_client_driver/go2_api.hpp"

Go2Api::Go2Api(rclcpp::Node *node) : node_(node) {
  RCLCPP_INFO(node_->get_logger(), "\033[1;34mGO2 API NODE INIT\033[0m");
  req_pub_ = node_->create_publisher<unitree_api::msg::Request>(
    "/api/sport/request", 10);
  RCLCPP_INFO(node_->get_logger(), "\033[1;34mINIT DONE.\033[0m");
  log_ = true;
}

void Go2Api::Damp(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_DAMP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mDAMP\033[0m");
  }
  req.header.identity.api_id = API_ID_DAMP;
  req_pub_->publish(req);
}

void Go2Api::BalanceStand(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_BALANCESTAND) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mBALANCESTAND\033[0m");
  }
  req.header.identity.api_id = API_ID_BALANCESTAND;
  req_pub_->publish(req);
}

void Go2Api::StopMove(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_STOPMOVE) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSTOPMOVE\033[0m");
  }
  req.header.identity.api_id = API_ID_STOPMOVE;
  req_pub_->publish(req);
}

void Go2Api::StandUp(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_STANDUP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSTANDUP\033[0m");
  }
  req.header.identity.api_id = API_ID_STANDUP;
  req_pub_->publish(req);
}

void Go2Api::StandDown(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_STANDDOWN) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSTANDDOWN\033[0m");
  }
  req.header.identity.api_id = API_ID_STANDDOWN;
  req_pub_->publish(req);
}

void Go2Api::RecoveryStand(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_RECOVERYSTAND) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mRECOVERY STAND\033[0m");
  }
  req.header.identity.api_id = API_ID_RECOVERYSTAND;
  req_pub_->publish(req);
}

void Go2Api::Euler(
  unitree_api::msg::Request& req,
  float roll, float pitch, float yaw
) {
  if (req.header.identity.api_id != API_ID_EULER) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mEULER\033[0m");
  }
  nlohmann::json js;
  js["x"] = roll;
  js["y"] = pitch;
  js["z"] = yaw;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_EULER;
  req_pub_->publish(req);
}

void Go2Api::Move(
  unitree_api::msg::Request& req,
  float vx, float vy, float vyaw
) {
  if (req.header.identity.api_id != API_ID_MOVE) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mMOVE\033[0m");
  }
  nlohmann::json js;
  js["x"] = vx;
  js["y"] = vy;
  js["z"] = vyaw;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_MOVE;
  req_pub_->publish(req);
}

void Go2Api::Sit(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_SIT) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSIT\033[0m");
  }
  req.header.identity.api_id = API_ID_SIT;
  req_pub_->publish(req);
}

void Go2Api::RiseSit(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_RISESIT) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mRISE SIT\033[0m");
  }
  req.header.identity.api_id = API_ID_RISESIT;
  req_pub_->publish(req);
}

void Go2Api::SpeedLevel(
  unitree_api::msg::Request& req,
  int level
) {
  if (req.header.identity.api_id != API_ID_SPEEDLEVEL) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSPEED LEVEL\033[0m");
  }
  nlohmann::json js;
  js["data"] = level;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_SPEEDLEVEL;
  req_pub_->publish(req);
}

void Go2Api::Hello(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_HELLO) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mHELLO\033[0m");
  }
  req.header.identity.api_id = API_ID_HELLO;
  req_pub_->publish(req);
}

void Go2Api::Stretch(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_STRETCH) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSTRETCH\033[0m");
  }
  req.header.identity.api_id = API_ID_STRETCH;
  req_pub_->publish(req);
}

void Go2Api::SwitchJoysitck(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_SWITCHJOYSTICK) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSWITCH JOYSTICK\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_SWITCHJOYSTICK;
  req_pub_->publish(req);
}

void Go2Api::Content(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_CONTENT) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mCONTENT\033[0m");
  }
  req.header.identity.api_id = API_ID_CONTENT;
  req_pub_->publish(req);
}

void Go2Api::Pose(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_POSE) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mPOSE\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_POSE;
  req_pub_->publish(req);
}

void Go2Api::Scrape(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_SCRAPE) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSCRAPE\033[0m");
  }
  req.header.identity.api_id = API_ID_SCRAPE;
  req_pub_->publish(req);
}

void Go2Api::FrontFlip(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_FRONTFLIP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFRONT FLIP\033[0m");
  }
  req.header.identity.api_id = API_ID_FRONTFLIP;
  req_pub_->publish(req);
}

void Go2Api::FrontJump(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_FRONTJUMP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFRONT JUMP\033[0m");
  }
  req.header.identity.api_id = API_ID_FRONTJUMP;
  req_pub_->publish(req);
}

void Go2Api::Dance1(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_DANCE1) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mDANCE 1\033[0m");
  }
  req.header.identity.api_id = API_ID_DANCE1;
  req_pub_->publish(req);
}

void Go2Api::Dance2(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_DANCE2) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mDANCE 2\033[0m");
  }
  req.header.identity.api_id = API_ID_DANCE2;
  req_pub_->publish(req);
}

void Go2Api::Heart(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_HEART) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mHEART\033[0m");
  }
  req.header.identity.api_id = API_ID_HEART;
  req_pub_->publish(req);
}

void Go2Api::StaticWalk(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_STATICWALK) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSTATIC WALK\033[0m");
  }
  req.header.identity.api_id = API_ID_STATICWALK;
  req_pub_->publish(req);
}

void Go2Api::TrotRun(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_TROTRUN) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mTROT RUN\033[0m");
  }
  req.header.identity.api_id = API_ID_TROTRUN;
  req_pub_->publish(req);
}

void Go2Api::EconomicGait(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_ECONOMICGAIT) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mECONOMIC GAIO\033[0m");
  }
  req.header.identity.api_id = API_ID_ECONOMICGAIT;
  req_pub_->publish(req);
}

void Go2Api::LeftFlip(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_LEFTFLIP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mLEFT FLIP\033[0m");
  }
  req.header.identity.api_id = API_ID_LEFTFLIP;
  req_pub_->publish(req);
}

void Go2Api::BackFilp(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_BACKFLIP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mBACKFLIP\033[0m");
  }
  req.header.identity.api_id = API_ID_BACKFLIP;
  req_pub_->publish(req);
}

void Go2Api::HandStand(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_HANDSTAND) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mHAND STAND\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_HANDSTAND;
  req_pub_->publish(req);
}

void Go2Api::FreeWalk(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_FREEWALK) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFREE WALK\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_FREEWALK;
  req_pub_->publish(req);
}

void Go2Api::FreeBound(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_FREEBOUND) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFREE BOUND\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_FREEBOUND;
  req_pub_->publish(req);
}

void Go2Api::FreeJump(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_FREEJUMP) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFREE JUMP\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_FREEJUMP;
  req_pub_->publish(req);
}

void Go2Api::FreeAvoid(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_FREEAVOID) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mFREE AVOID\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_FREEAVOID;
  req_pub_->publish(req);
}

void Go2Api::ClassicWalk(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_CLASSICWALK) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mCLASSIC WALK\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_CLASSICWALK;
  req_pub_->publish(req);
}

void Go2Api::WalkUpgright(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_WALKUPRIGHT) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mWALK UPRIGHT\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_WALKUPRIGHT;
  req_pub_->publish(req);
}

// NOTICE: AutoRecoveryGet is only avaliable on ROS2 Humble
void Go2Api::AutoRecoveryGet(
  unitree_api::msg::Request& req,
  bool flag
) {
  if (req.header.identity.api_id != API_ID_AUTORECOVERY_GET) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mAUTO RECOVERY GET\033[0m");
  }
  nlohmann::json js;
  js["data"] = flag;
  req.parameter = js.dump();
  req.header.identity.api_id = API_ID_AUTORECOVERY_GET;
  req_pub_->publish(req);
}

void Go2Api::SwitchAvoidMode(unitree_api::msg::Request& req) {
  if (req.header.identity.api_id != API_ID_SWITCHAVOIDMODE) {
    RCLCPP_INFO(node_->get_logger(), "GO2 SPORT MODE: \033[1;34mSWITCH AVOID MODE\033[0m");
  }
  req.header.identity.api_id = API_ID_SWITCHAVOIDMODE;
  req_pub_->publish(req);
}