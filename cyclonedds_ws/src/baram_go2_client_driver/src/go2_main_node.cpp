#include "baram_go2_client_driver/go2_main_node.hpp"

// rclcpp::Node를 상속받아 클래스를 정의합니다.

class MainNode : public rclcpp::Node
{
public:
  // 생성자
  MainNode()
  : Node("main_node") // 노드 이름 초기화
  {
    //IMU 구독자 설정
      imu_suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
        "lf/sportmodestate", 1,
        [this](const unitree_go::msg::SportModeState::SharedPtr data) {
          HighStateHandler(data);
        });

    //나머지 모드 변경 플래그를 받을 sub
    start_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/start_mode", 10, std::bind(&MainNode::start_mode_callback, this, std::placeholders::_1));
    
    //depth flag를 받을 sub
    depth_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/depth_flag", 10, std::bind(&MainNode::depth_mode_callback, this, std::placeholders::_1));

    //yolo 플래그를 받을 sub
    yolo_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "/realsence/traffic_detection", 10, std::bind(&MainNode::yolo_callback, this, std::placeholders::_1));

    // running client 생성
    running_client_ = this->create_client<baram_go2_interfaces::srv::Straight>("/straight_control");
  }

private:
  
  //get init yaw data
  void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
    if (init_yaw == -100){
      state_ = *msg;
    
      // RCLCPP_INFO(this->get_logger(), "Position: %f, %f, %f", state_.position[0],
      //             state_.position[1], state_.position[2]);
      // RCLCPP_INFO(this->get_logger(), "IMU rpy: %f, %f, %f",
      //             state_.imu_state.rpy[0], state_.imu_state.rpy[1],
      //             state_.imu_state.rpy[2]);

      init_yaw = state_.imu_state.rpy[2];
      RCLCPP_INFO(this->get_logger(), "init_imu %f", init_yaw);
    }
  }

  //start mode에 따른 flag 처리
  void start_mode_callback(const std_msgs::msg::Int16 & msg)
  {
    //터미널로 0을 쏴 주면 시작한다.
    if (msg.data == static_cast<int16_t>(main_node_mode::START)){
      current_mode = main_node_mode::START;

    }
    else if (msg.data == static_cast<int16_t>(main_node_mode::PORTHOLE)){
      porthole_request(init_yaw + 1.5* PI, 20);
    }
  }

  void depth_mode_callback(const std_msgs::msg::Int16 & msg)
  {
    //BEFORETRAFFIC 단계이고, depth값이 들어왔다고 한다면, 실행한다.
    if (current_mode == main_node_mode::BEFORETRAFFIC && msg.data == 1){
      
      // 1. 서비스 요청(request) 객체 생성
      auto request = std::make_shared<baram_go2_interfaces::srv::Straight::Request>();
      request->angle = init_yaw + PI;
      //직진
      request->mode = 2;
      //달리기 모드로 변경
      request->run_mode = 0;

      // 2. 비동기적으로 서비스 요청을 보내고, 응답(future)을 기다립니다.
      //    응답이 오면 지정된 람다(lambda) 함수가 실행됩니다.
      auto response_callback = [this](rclcpp::Client<baram_go2_interfaces::srv::Straight>::SharedFuture future) {
        // 3. 서비스 응답(response) 처리
        auto result = future.get();
        RCLCPP_INFO(this->get_logger(), "서비스 응답 수신: %d",
                    result->success);

        //종료 flag
        current_mode = main_node_mode::TRAFFIC;
      };

      running_client_->async_send_request(request, response_callback);
      RCLCPP_INFO(this->get_logger(), "서비스 요청 전송: %f + %d", request->angle, request->run_mode);
    }
  }

  // traffic 신호를 받으면 service client 출력을 보낼 것이다.
  void yolo_callback(const std_msgs::msg::Int16 & msg)
  {
    RCLCPP_INFO(this->get_logger(), "traffic comand 메시지: %d", msg.data);

    //초록 신호가 들어오고, 모드가 변경된 상황이면
    if (msg.data == 0 && current_mode == main_node_mode::TRAFFIC){
      // 서비스 서버에 요청을 보냅니다.
      double target_angle = init_yaw + PI;
      double target_time = 10;
      this->running_request(target_angle, target_time);
    }
    // 초록불이 아니면 멈춰있어야 한다.
    else {

    }
  }

  //port hole을 위한  직빨
  void porthole_request(double target_angle, double time) 
  {
    // 1. 서비스 요청(request) 객체 생성
    auto request = std::make_shared<baram_go2_interfaces::srv::Straight::Request>();
    request->angle = target_angle;
    //직진
    request->mode = 1;
    request->velocity = 0.3;
    //달리기 모드로 변경
    request->run_mode = 0;
    request->end_time = time;

    // 2. 비동기적으로 서비스 요청을 보내고, 응답(future)을 기다립니다.
    //    응답이 오면 지정된 람다(lambda) 함수가 실행됩니다.
    auto response_callback = [this](rclcpp::Client<baram_go2_interfaces::srv::Straight>::SharedFuture future) {
      // 3. 서비스 응답(response) 처리
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "서비스 응답 수신: %d",
                  result->success);

      //종료 flag
      current_mode = main_node_mode::BEFORETRAFFIC;
    };

    running_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(this->get_logger(), "서비스 요청 전송: %f + %d", request->angle, request->run_mode);
  }


  // --- 마지막 질주 함수 ---
  void running_request(double target_angle, double time) 
  {
    // 1. 서비스 요청(request) 객체 생성
    auto request = std::make_shared<baram_go2_interfaces::srv::Straight::Request>();
    request->angle = target_angle;
    //직진
    request->mode = 1;
    request->velocity = 0.3;
    //달리기 모드로 변경
    request->run_mode = 1;
    request->end_time = time;

    // 2. 비동기적으로 서비스 요청을 보내고, 응답(future)을 기다립니다.
    //    응답이 오면 지정된 람다(lambda) 함수가 실행됩니다.
    auto response_callback = [this](rclcpp::Client<baram_go2_interfaces::srv::Straight>::SharedFuture future) {
      // 3. 서비스 응답(response) 처리
      auto result = future.get();
      RCLCPP_INFO(this->get_logger(), "서비스 응답 수신: %d",
                  result->success);

      //종료 flag
      current_mode = main_node_mode::END;
    };

    running_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(this->get_logger(), "서비스 요청 전송: %f + %d", request->angle, request->run_mode);
  }

  // 멤버 변수 선언
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr start_sub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr depth_sub_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr yolo_sub_;
  rclcpp::Client<baram_go2_interfaces::srv::Straight>::SharedPtr running_client_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr imu_suber_;
  unitree_go::msg::SportModeState state_;

  double init_yaw = -100;
  main_node_mode current_mode;
};

// --- main 함수 ---
int main(int argc, char * argv[])
{
  // ROS 2 초기화
  rclcpp::init(argc, argv);
  // MainNode 객체를 생성하고 ROS 2 네트워크에서 실행(spin)
  rclcpp::spin(std::make_shared<MainNode>());
  // ROS 2 종료
  rclcpp::shutdown();
  return 0;
}