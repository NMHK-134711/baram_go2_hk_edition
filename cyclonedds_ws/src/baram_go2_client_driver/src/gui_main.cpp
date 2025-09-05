#include <QApplication>
#include "baram_go2_client_driver/gui_main_window.hpp"

int main(int argc, char *argv[])
{
  // ROS2 와 Qt 초기화
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  // 메인 윈도우 생성 및 표시
  MainWindow w;
  w.show();

  // Qt 애플리케이션 실행 (이벤트 루프 시작)
  int result = app.exec();

  // 종료
  rclcpp::shutdown();
  return result;
}