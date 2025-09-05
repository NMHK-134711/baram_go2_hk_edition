#include "baram_go2_client_driver/go2_main_node.hpp"

// rclcpp::Node를 상속받아 클래스를 정의합니다.

class MainNode : public rclcpp::Node
{
public:
  // 생성자
  MainNode()
  : Node("start_node") // 노드 이름 초기화
  {
    //input service 
    service_ = this->create_service<baram_go2_interfaces::srv::Straight>("straight_control",
      std::bind(&StraightControl::straight_command, this, std::placeholders::_1, std::placeholders::_2));

    // running client 생성
    running_client_ = this->create_client<baram_go2_interfaces::srv::Straight>("straight_control");
  }

private:

  void straight_command(
    const std::shared_ptr<baram_go2_interfaces::srv::Straight::Request> request,
    std::shared_ptr<baram_go2_interfaces::srv::Straight::Response>      response)
  {
    //입력 데이터를 받는다.
    double target_angle = request->angle;
    double time = request->end_time;
    
    //running에 대한 request와, 이 server에 대한 response도 같이 넘겨준다.
    running_request(target_angle, time, response);
  }

  // --- 초반 질주 함수 ---
  void running_request(double target_angle, double time, std::shared_ptr<baram_go2_interfaces::srv::Straight::Response> original_response) 
  {
    // 1. 서비스 요청(request) 객체 생성
    auto request = std::make_shared<baram_go2_interfaces::srv::Straight::Request>();
    request->angle = target_angle;
    //직진
    request->mode = 1;
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
    };

    running_client_->async_send_request(request, response_callback);
    RCLCPP_INFO(this->get_logger(), "서비스 요청 전송: %f + %d", request->angle, request->run_mode);
  }

  // 멤버 변수 선언
  rclcpp::Service<baram_go2_interfaces::srv::Straight>::SharedPtr service_;
  rclcpp::Client<baram_go2_interfaces::srv::Straight>::SharedPtr running_client_;

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