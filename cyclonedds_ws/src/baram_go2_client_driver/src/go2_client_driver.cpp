/**
 * Author: Kho Geonwoo | Jang Seungwon | Lee Jinwoo
 *
 * REFERENCE: unitre_ros2, go2_driver
 * URL: https://github.com/unitreerobotics/unitree_ros2/tree/master
 */

#include "baram_go2_client_driver/go2_client_driver.hpp"

// #define DEBUG_POS_
// #define DEBUG_CRAWL_
// #define DEBUG_IMU_

// #define DEBUG_T1_

Go2HighControl::Go2HighControl()
 : Node("go2_high_control"),
   api_client_(this), api_slam_(this),
   robot_state_client_(this), motion_switcher_client_(this) {
  RCLCPP_INFO(this->get_logger(), "\033[1;34mGO2 HIGH CONTROL INIT\033[0m");

  mode_.store(STAND_UP);

  last_act_timer_ = std::chrono::steady_clock::now();
  timer_ = this->create_wall_timer(std::chrono::seconds(10),
    [this] {
      if (mode_.load() == STAND_UP || mode_.load() == BALANCE_STAND) {
        auto tic = std::chrono::steady_clock::now();
        auto toc = std::chrono::duration_cast<std::chrono::seconds>(
          tic - last_act_timer_).count();
        if (toc >= 10) {
          RCLCPP_INFO(this->get_logger(),
            "IDLE 10s in NORMAL STAND -> \033[1;32mSTAND DOWN\033[0m");
          mode_.store(STAND_DOWN);
          ControlHandler();
          last_act_timer_ = tic;
        }
      }
    }
  );

  sportmode_state_sub_ = this->create_subscription<unitree_go::msg::SportModeState>(
    TOPIC_HIGHSTATE, 1, [this](const unitree_go::msg::SportModeState::SharedPtr data) {
      HighStateHandler(data);
  });

  low_state_sub_ = this->create_subscription<unitree_go::msg::LowState>(
    TOPIC_LOWSTATE, 10, [this](const unitree_go::msg::LowState::SharedPtr data) {
      LowStateHandler(data);
  });

  api_req_sub_ = this->create_subscription<unitree_api::msg::Request>(
    TOPIC_API_REQ, 1, [this](const unitree_api::msg::Request::SharedPtr data) {
      ApiReqHandler(data);
  });

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr data) {
      CmdVelHandler(data);
  });

  low_cmd_pub_ = this->create_publisher<unitree_go::msg::LowCmd>(TOPIC_LOWCMD, 10);

  SetInitState();

  // int32_t status;
  // robot_state_client_.ServiceSwitch("mcf", 1, status);

  crawl_state_ = CrawlState::WAIT;

  // TESTING SECTION (THREAD)
#ifdef DEBUG_T1_
  t1_ = std::thread([this] {
    RCLCPP_INFO(this->get_logger(), "\033[1;35mt1 thread\033[0m");
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 1) Switch Service
    // robot_state_client_.ServiceSwitch("mcf", 1, status_);

    ControlHandler();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // mode_.store(STAND_DOWN);
    // RCLCPP_INFO(this->get_logger(), "\033[1;35mt1 thread: stand down\033[0m");
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    // mode_.store(DAMP);
    // RCLCPP_INFO(this->get_logger(), "\033[1;35mt1 thread: damp\033[0m");
    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // mode_.store(CRAWLING);
    // RCLCPP_INFO(this->get_logger(), "\033[1;35mt1 thread: crawling\033[0m");
    std::string test_dir = "/home/go/unitree_ros2/map/map.pcd";
    // std::string test_dir = "/home/unitree/map/map.pcd";
    RCLCPP_INFO(this->get_logger(), "\033[1;35mStart Mapping for 15s.\033[0m\nsave dir: %s", test_dir.c_str());
    try {
      api_slam_.StartMapping(api_req_);
      // RCLCPP_INFO(this->get_logger(), "status: \033[1;35m%u\033[0m", status_);
    } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed[SLAM]: %s", e.what());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(15000));
    api_slam_.EndMapping(api_req_, test_dir);
    RCLCPP_INFO(this->get_logger(), "\033[1;35mEnd Mapping.\033[0m");
    // ControlHandler();
  });
#endif // DEBUG_T1_

}

void Go2HighControl::SwitchLowCmd() {
  RCLCPP_INFO(this->get_logger(), "Change mode to \033[1;32mLOW CONTROL MODE\033[0m");

  motion_switcher_client_.ReleaseMode(api_req_);
  low_cmd_.head[0] = 0xFE;
  low_cmd_.head[1] = 0xEF;
  low_cmd_.level_flag = LOWLEVEL; // 0xFF
  low_cmd_.gpio = 0;

  for (int i = 0; i < 20; ++i) {
    low_cmd_.motor_cmd[i].mode = (0x01);  // motor switch to servo (PMSM) mode
    low_cmd_.motor_cmd[i].q = (PosStopF);
    low_cmd_.motor_cmd[i].kp = (0);
    low_cmd_.motor_cmd[i].dq = (VelStopF);
    low_cmd_.motor_cmd[i].kd = (0);
    low_cmd_.motor_cmd[i].tau = (0);
  }
  RCLCPP_INFO(this->get_logger(), "\033[1;31mLOW CONTORL INIT DONE\033[0m");
}

void Go2HighControl::SwitchHighCmd() {
  RCLCPP_INFO(this->get_logger(), "\033[1;34mHIGH CONTROL MODE\033[0m");

  std::thread([this] {
    RCLCPP_INFO(this->get_logger(), "Switch service to \033[1;34mMFC\033[0m");
    robot_state_client_.ServiceSwitch("mcf", 1, status_);
    RCLCPP_INFO(this->get_logger(), "Switched: \033[1;34mMFC\033[0m");
    // mode_.store(STAND_UP);
    // RCLCPP_INFO(this->get_logger(), "STAND UP stored in mode_");
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    // ControlHandler();
    mode_.store(BALANCE_STAND);
    RCLCPP_INFO(this->get_logger(), "BALANCE STAND stored in mode_");
  }).detach();
}

void Go2HighControl::SetInitState() {
  px0_ = state_.position[0];
  py0_ = state_.position[1];
  yaw0_ = state_.imu_state.rpy[2];

  RCLCPP_INFO(
    this->get_logger(), "\n\033[1;32mSet initial position:\n\tx0: %f\n\ty0: %f\n\tyaw0: %f\033[0m",
    px0_, py0_,yaw0_
  );
}

void Go2HighControl::GetInitState() {
  RCLCPP_INFO(
    this->get_logger(), "\n\033[1;32mGet initial position:\n\tx0: %f\n\ty0: %f\n\tyaw0: %f\033[0m",
    px0_, py0_,yaw0_
  );
}

void Go2HighControl::InitCrawlPos() {
  RCLCPP_INFO(this->get_logger(), "Init target poses for \033[1;34mCRAWLING\033[0m");
  crawl_target_pos_.clear();
  crawl_target_pos_.reserve(12);
  crawl_target_pos_.push_back(0.0);
  crawl_target_pos_.push_back(1.15);
  crawl_target_pos_.push_back(-2.3);
  crawl_target_pos_.push_back(0.0);
  crawl_target_pos_.push_back(1.15);
  crawl_target_pos_.push_back(-2.3);
  crawl_target_pos_.push_back(-0.2);
  crawl_target_pos_.push_back(1.15);
  crawl_target_pos_.push_back(-2.3);
  crawl_target_pos_.push_back(0.2);
  crawl_target_pos_.push_back(1.15);
  crawl_target_pos_.push_back(-2.3);
}

void Go2HighControl::HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
  state_ = *msg;

  if (!low_loop_active_) ControlHandler();
  else RCLCPP_WARN(this->get_logger(), "Low Loop activated. Cannot service High Level Control");

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
}

void Go2HighControl::LowStateHandler(const unitree_go::msg::LowState::SharedPtr msg) {
  low_state_ = *msg;
}

void Go2HighControl::ApiReqHandler(const unitree_api::msg::Request::SharedPtr msg){
  api_req_ = *msg;
}

void Go2HighControl::CmdVelHandler(const geometry_msgs::msg::Twist::SharedPtr msg) {
  constexpr double EPS = 1e-4;
  const bool empty_cmd =
      std::fabs(msg->linear.x)  < EPS &&
      std::fabs(msg->linear.y)  < EPS &&
      std::fabs(msg->angular.z) < EPS;

  last_act_timer_ = std::chrono::steady_clock::now();
  cmd_vel_ = *msg;

  if (empty_cmd) {
    if (mode_.load() == NAVIGATION) {
      std::scoped_lock lk(mtx_);
      RCLCPP_INFO(this->get_logger(), "Idle cmd in NAVGATION mode. Change it to BALANCE STAND mode.");
      try {
        api_client_.Move(api_req_, 0.0, 0.0, 0.0);
      } catch (const std::exception &e) {
        RCLCPP_WARN(this->get_logger(), "Failed[NAVIGATION]: %s", e.what());
      }

      if (mode_.load() != BALANCE_STAND) {
        try {
          mode_.store(BALANCE_STAND);
          api_client_.BalanceStand(api_req_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "Failed[BALANCE_STAND]: %s", e.what());
        }
      }
    }
    return;
  }

  {
    std::scoped_lock lk(mtx_);

    // TODO: Add delays btw each clien's call method if needed.
    if (mode_.load() == STAND_DOWN) {
      // RCLCPP_INFO(this->get_logger(), "304");
      mode_.store(STAND_UP);
      api_client_.StandUp(api_req_);
    }
    if (mode_.load() == STAND_UP) {
      // RCLCPP_INFO(this->get_logger(), "309");
      mode_.store(BALANCE_STAND);
      api_client_.BalanceStand(api_req_);
    }
    if (mode_.load() == BALANCE_STAND) {
      // RCLCPP_INFO(this->get_logger(), "314: %f %f %f",cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z);
      // api_client_.BalanceStand(api_req_);
      api_client_.Move(api_req_, cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z);
      mode_.store(NAVIGATION);
    }
  }
}

void Go2HighControl::ControlHandler() {
  switch (mode_.load()) {
    case STAND_UP:
      api_client_.StandUp(api_req_);
      break;
    case BALANCE_STAND:
      api_client_.BalanceStand(api_req_);
      break;
    case NAVIGATION:
      api_client_.Move(
        api_req_,
        cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z
      );
      break;
    case JUMP_FORWARD:
      api_client_.FrontJump(api_req_);
      break;
    case DAMP:
      api_client_.Damp(api_req_);
      break;
    case STAND_DOWN:
      api_client_.StandDown(api_req_);
      break;
    case STOP_MOVE:
      api_client_.StopMove(api_req_);
      break;
    case CRAWLING:
      if (crawl_.exchange(true)) break;
      Crawl();
      break;
    default:
      api_client_.StopMove(api_req_);
  }
}

void Go2HighControl::LowCmdWrite() {
  auto set_cmd =
  [&](int i, double q, double dq, double kp, double kd, double tau) {
    low_cmd_.motor_cmd[i].q = static_cast<float>(q);
    low_cmd_.motor_cmd[i].dq = static_cast<float>(dq);
    low_cmd_.motor_cmd[i].kp = static_cast<float>(kp);
    low_cmd_.motor_cmd[i].kd = static_cast<float>(kd);
    low_cmd_.motor_cmd[i].tau = static_cast<float>(tau);
    low_cmd_.motor_cmd[i].mode = 0x01;
  };

  switch (crawl_state_) {
    case CrawlState::WAIT: {
      #ifdef DEBUG_CRAWL_
        RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWL: WAIT\033[0m");
      #endif
      if (low_state_.motor_state.size() < 12) return;
      for (int i=0; i<12; i++) crawl_start_pos_[i] = low_state_.motor_state[i].q;
      crawl_align_percent_ = 0.0;
      crawl_state_ = CrawlState::ALIGN;
      break;
    }
    case CrawlState::ALIGN: {
      #ifdef DEBUG_CRAWL_
        RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWL: ALIGN\033[0m");
      #endif
      crawl_align_percent_ += 1.0 / static_cast<double>(duration_align_);
      if (crawl_align_percent_ > 1.0) crawl_align_percent_ = 1.0;

      for (int i=0; i<12; i++) {
        double q_target = crawl_target_pos_[i];
        double q = (1.0 - crawl_align_percent_) * crawl_start_pos_[i] + crawl_align_percent_ * q_target;
        set_cmd(i, q, 0.0, Kp_, Kd_, 0.0);
      }
      if (crawl_align_percent_ >= 1.0) {
        crawl_state_ = CrawlState::TRANSITION;
        transition_step_ = 0;
        trot_step_ = 0;
      }
      break;
    }
    case CrawlState::TRANSITION: {
      #ifdef DEBUG_CRAWL_
        RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWL: TRANSITION\033[0m");
      #endif
        transition_step_++;
      trot_step_ = (trot_step_ + 1) % trot_duration_;

      double ramp = std::min(1.0, static_cast<double>(transition_step_) / static_cast<double>(transition_duration_));
      double tnorm = static_cast<double>(trot_step_) / static_cast<double>(trot_duration_);

      for (int i=0; i<12; i++) {
        double q_trot = EvalCatmullRomClosed(joint_control_pts_[i], tnorm);
        double q_stand = crawl_target_pos_[i];
        double q = q_stand + ramp * (q_trot - q_stand);
        set_cmd(i, q, 0.0, Kp_, Kd_, 0.0);
      }
      if (transition_step_ >= transition_duration_) crawl_state_ = CrawlState::TROT;
      break;
    }
    case CrawlState::TROT: {
      #ifdef DEBUG_CRAWL_
        RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWL: TROT\033[0m");
        RCLCPP_INFO(this->get_logger(), "DURATION LEFT : \033[1;34m%d / %d\033[0m", trot_step_, trot_duration_);
      #endif
        trot_step_ = (trot_step_ + 1) % trot_duration_;
      double tnorm = static_cast<double>(trot_step_) / static_cast<double>(trot_duration_);

      for (int i=0; i<12; i++) {
        double q = EvalCatmullRomClosed(joint_control_pts_[i], tnorm);
        set_cmd(i, q, 0.0, Kp_, Kd_, 0.0);
      }
      if (trot_step_ >= trot_duration_ - 1)
        trot_cycle_cnt_++;
      // if (trot_step_ >= trot_duration_) crawl_state_ = CrawlState::DONE;
      if (trot_cycle_cnt_ >= 4) crawl_state_ = CrawlState::DONE;
      break;
    }
    case CrawlState::DONE: {
      #ifdef DEBUG_CRAWL_
        RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWL: DONE\033[0m");
      #endif
      StopCrawl();
      return;
    }
  }

  get_crc(low_cmd_);  // IMPORTANT
  low_cmd_pub_->publish(low_cmd_);
  #ifdef DEBUG_CRAWL_
    RCLCPP_INFO(this->get_logger(), "SUCCESSFULLY Publish Low commands\033[0m");
  #endif
}

void Go2HighControl::TrotSpline() {
  RCLCPP_INFO(this->get_logger(), "TROT SPLINE");
  const double base[4][12] = {
    {-0.0, 1.15, -2.3,   0.0, 1.15, -2.3,   -0.2, 1.20, -2.2,   0.2, 1.20, -2.2},
    {-0.2, 1.97, -2.4,   0.2, 1.97, -2.4,   -0.4, 1.90, -2.2,   0.4, 1.90, -2.2},
    {-0.3, 1.30, -2.7,   0.3, 1.30, -2.7,   -0.5, 1.10, -2.5,   0.5, 1.10, -2.5},
    {-0.1, 0.89, -2.37,  0.1, 0.90, -2.35,  -0.3, 0.82, -2.3,   0.3, 0.80, -2.3}
  };

  auto extract_leg = [&](int c) {
    std::vector<std::array<double, 3>> leg(4);
    for (int i = 0; i < 4; i++) {
      leg[i] = { base[i][c], base[i][c+1], base[i][c+2] };
    }
    return leg;
  };

  auto fr = extract_leg(0);
  auto fl = fr;
  for (int i = 0; i < 4; i++) {
    fl[i][0] = -fl[i][0];
  }

  auto roll2 = [](const std::vector<std::array<double, 3>>& in) {
    std::vector<std::array<double, 3>> o(4);
    o[0] = in[2]; o[1] = in[3]; o[2] = in[0]; o[3] = in[1];
    return o;
  };
  auto frR = roll2(fr);
  auto flR = roll2(fl);

  auto periodic = [](const std::vector<std::array<double, 3>>& in) {
    std::vector<std::array<double, 3>> o = in;
    o.push_back(in.front());
    return o;
  };

  auto frP = periodic(fr);
  auto flP = periodic(fl);
  auto frRP = periodic(frR);
  auto flRP = periodic(flR);

  std::vector<std::vector<double>> allKF(5, std::vector<double>(12, 0.0));
  for (int t = 0; t < 5; t++) {
    allKF[t][0]=frP[t][0];  allKF[t][1] =frP[t][1];  allKF[t][2] =frP[t][2];
    allKF[t][3]=flRP[t][0]; allKF[t][4] =flRP[t][1]; allKF[t][5] =flRP[t][2];
    allKF[t][6]=frRP[t][0]; allKF[t][7] =frRP[t][1]; allKF[t][8] =frRP[t][2];
    allKF[t][9]=flP[t][0];  allKF[t][10]=flP[t][1];  allKF[t][11]=flP[t][2];
  }

  for (int i = 0; i < 12; i++){
    joint_control_pts_[i].clear();
    joint_control_pts_[i].reserve(5);
    for (int t = 0; t < 5; t++) joint_control_pts_[i].push_back(allKF[t][i]);
  }
}

double Go2HighControl::EvalCatmullRomClosed(const std::vector<double>& p, double t01) const {
  const int n = static_cast<int>(p.size());
  if (n == 0) return 0.0;
  if (n == 1) return p[0];

  const int segs = std::max(1, n - 1);
  double t = t01 - std::floor(t01);   // [0,1)
  double x = t * segs;
  int i = static_cast<int>(std::floor(x));
  if (i >= segs) i = segs - 1;
  double u = x - i;

  auto m = [n](int k) { return (k%n + n) % n; };
  int i0 = m(i - 1), i1 = m(i), i2 = m(i + 1), i3 = m(i + 2);
  double p0 = p[i0], p1 = p[i1], p2 = p[i2], p3 = p[i3];

  double u2 = u*u, u3 = u2*u;
  return 0.5 * ((2.0 * p1) + (-p0 + p2) * u +
                (2.0 * p0 - 5.0 * p1 + 4.0 * p2 - p3) * u2 +
                (-p0 + 3.0 * p1 - 3.0 * p2 + p3) * u3);
}

void Go2HighControl::Crawl() {
  if (low_loop_active_) return;

  crawl_state_ = CrawlState::WAIT;
  crawl_align_percent_ = 0.0;
  transition_step_ = 0;
  trot_step_ = 0;

  SwitchLowCmd();
  InitCrawlPos();
  TrotSpline();

  RCLCPP_INFO(this->get_logger(), "\033[1;32mCRAWLING START\033[0m");
  low_loop_active_ = true;

  low_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(2), [this] { this->LowCmdWrite(); }
  );

}

void Go2HighControl::StopCrawl() {
  if (!low_loop_active_) return;

  mode_.store(BALANCE_STAND);
  if (low_loop_timer_) low_loop_timer_.reset();

  low_loop_active_ = false;
  crawl_.store(false);

  RCLCPP_INFO(this->get_logger(), "\033[1;31mCRAWLING DONE. \033[1;34mSWITCH CONTROL MODE.\033[0m");
  SwitchHighCmd();
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Go2HighControl>();
  node->GetInitState();

  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();

  return 0;
}