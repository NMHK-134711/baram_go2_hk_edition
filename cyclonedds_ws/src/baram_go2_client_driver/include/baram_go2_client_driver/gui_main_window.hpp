#ifndef MAIN_WINDOW_HPP_
#define MAIN_WINDOW_HPP_

#include <QMainWindow>
#include <QProcess>
#include <rclcpp/rclcpp.hpp>
#include <unitree_go/msg/sport_mode_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <QString>
#include <mutex>
#include <deque> // 이동 평균 필터를 위해 deque 헤더 추가

#include "ui_data_browser.h"

class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

signals:
  void update_gui();

private slots:
  void on_update_gui();
  void on_topic_list_button_clicked();
  void read_topic_list_output();

private:
  void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg);
  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  Ui::MainWindow *ui;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
  // PointCloud2를 사용하도록 다시 변경
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;

  QString latest_state_text_;
  QString latest_lidar_text_;
  std::mutex data_mutex_;
  
  QProcess *topic_process_;

  // --- 이동 평균 필터를 위한 멤버 변수들 ---
  const size_t filter_size_ = 5; // 평균을 낼 데이터 개수
  std::deque<float> front_dist_buffer_;
  std::deque<float> front_left_dist_buffer_;
  std::deque<float> front_right_dist_buffer_;
  std::deque<float> left_dist_buffer_;
  std::deque<float> right_dist_buffer_;
};

#endif // MAIN_WINDOW_HPP_