#include <thread>
#include <numeric> // std::accumulate를 위해 추가
#include "baram_go2_client_driver/gui_main_window.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>

MainWindow::MainWindow(QWidget *parent) :
  QMainWindow(parent),
  ui(new Ui::MainWindow)
{
  ui->setupUi(this);
  ros_node_ = rclcpp::Node::make_shared("go2_gui_node");

  subscription_ = ros_node_->create_subscription<unitree_go::msg::SportModeState>(
    "lf/sportmodestate", 10,
    std::bind(&MainWindow::state_callback, this, std::placeholders::_1));

  lidar_subscription_ = ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud", 10,
    std::bind(&MainWindow::lidar_callback, this, std::placeholders::_1));

  connect(this, &MainWindow::update_gui, this, &MainWindow::on_update_gui);
  connect(ui->topic_list_button, &QPushButton::clicked, this, &MainWindow::on_topic_list_button_clicked);
  topic_process_ = new QProcess(this);
  connect(topic_process_, &QProcess::readyReadStandardOutput, this, &MainWindow::read_topic_list_output);

  for(size_t i = 0; i < filter_size_; ++i) {
    front_dist_buffer_.push_back(10.0f);
    front_left_dist_buffer_.push_back(10.0f);
    front_right_dist_buffer_.push_back(10.0f);
    left_dist_buffer_.push_back(10.0f);
    right_dist_buffer_.push_back(10.0f);
  }

  std::thread([this]() {
    rclcpp::spin(ros_node_);
  }).detach();

  RCLCPP_INFO(ros_node_->get_logger(), "GUI Node has been started.");
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::state_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
{
    QString text = QString("Position:\n  X: %1\n  Y: %2\n  Z: %3\n\n"
                         "Velocity:\n  VX: %4\n  VY: %5\n  VZ: %6\n\n"
                         "IMU RPY (Roll, Pitch, Yaw):\n  R: %7\n  P: %8\n  Y: %9\n\n"
                         "IMU Quaternion:\n  X: %10\n  Y: %11\n  Z: %12\n  W: %13")
    .arg(msg->position[0]).arg(msg->position[1]).arg(msg->position[2])
    .arg(msg->velocity[0]).arg(msg->velocity[1]).arg(msg->velocity[2])
    .arg(msg->imu_state.rpy[0]).arg(msg->imu_state.rpy[1]).arg(msg->imu_state.rpy[2])
    .arg(msg->imu_state.quaternion[0]).arg(msg->imu_state.quaternion[1])
    .arg(msg->imu_state.quaternion[2]).arg(msg->imu_state.quaternion[3]);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_state_text_ = text;
    }
    emit update_gui();
}

// 최종 수정된 lidar_callback 함수
void MainWindow::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    float min_front_dist = 10.0f;
    float min_left_dist = 10.0f;
    float min_right_dist = 10.0f;

    sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;
        
        // ==================== Z축 필터 수정 ====================
        // 로봇 몸통(Z > 0)을 확실히 제외하고, 바닥 근처(-15cm ~ +15cm)의 장애물만 보도록 범위를 좁힙니다.
        if (z < -0.15f || z > 0.15f) { 
            continue; 
        }
        // =======================================================

        float dist = std::sqrt(x * x + y * y);
        if (dist < 0.2f) { continue; }

        float angle_deg = std::atan2(y, x) * 180.0 / M_PI;

        // 정면 영역 (-45도 ~ +45도)
        if (angle_deg > -45.0f && angle_deg <= 45.0f) {
            if (dist < min_front_dist) {
                min_front_dist = dist;
            }
        }
        // 좌측 영역 (45도 ~ 135도)
        else if (angle_deg > 45.0f && angle_deg <= 135.0f) {
            if (dist < min_left_dist) {
                min_left_dist = dist;
            }
        }
        // 우측 영역 (-135도 ~ -45도)
        else if (angle_deg > -135.0f && angle_deg <= -45.0f) {
            if (dist < min_right_dist) {
                min_right_dist = dist;
            }
        }
    }

    // --- 필터 로직 (front, left, right 3개로 단순화) ---
    front_dist_buffer_.push_back(min_front_dist);
    if(front_dist_buffer_.size() > filter_size_) front_dist_buffer_.pop_front();
    
    left_dist_buffer_.push_back(min_left_dist);
    if(left_dist_buffer_.size() > filter_size_) left_dist_buffer_.pop_front();

    right_dist_buffer_.push_back(min_right_dist);
    if(right_dist_buffer_.size() > filter_size_) right_dist_buffer_.pop_front();

    // 버퍼에 저장된 값들의 평균 계산
    float avg_front = std::accumulate(front_dist_buffer_.begin(), front_dist_buffer_.end(), 0.0) / front_dist_buffer_.size();
    float avg_left = std::accumulate(left_dist_buffer_.begin(), left_dist_buffer_.end(), 0.0) / left_dist_buffer_.size();
    float avg_right = std::accumulate(right_dist_buffer_.begin(), right_dist_buffer_.end(), 0.0) / right_dist_buffer_.size();
    // --------------------------------------------------

    QString lidar_text = QString("LiDAR Distances (Final & Filtered):\n"
                                 "  Front: %1 m\n"
                                 "  Left:  %2 m\n"
                                 "  Right: %3 m")
                         .arg(avg_front, 0, 'f', 2)
                         .arg(avg_left, 0, 'f', 2)
                         .arg(avg_right, 0, 'f', 2);
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_lidar_text_ = lidar_text;
    }
    emit update_gui();
}

// 이 슬롯은 메인 GUI 스레드에서 실행됩니다.
void MainWindow::on_update_gui()
{
  // 멤버 변수에 저장된 최신 텍스트를 조합하여 한 번에 업데이트
  std::lock_guard<std::mutex> lock(data_mutex_);
  ui->sensor_data_viewer->setText(latest_state_text_ + "\n\n" + latest_lidar_text_);
}

// 버튼 클릭 시 호출될 슬롯 구현
void MainWindow::on_topic_list_button_clicked()
{
  ui->topic_list_viewer->clear(); // 텍스트 브라우저를 먼저 비웁니다.

  // ROS 환경을 source하고 ros2 topic list 명령어를 실행
  QString command = "bash";
  QStringList args;
  args << "-c" << "source /opt/ros/humble/setup.bash && "
                    "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && "
                    "export CYCLONEDDS_URI=file:///home/hk/cyclonedds.xml && " // hk는 사용자 이름에 맞게 수정
                    "ros2 topic list";

  topic_process_->start(command, args);
}

// topic_process_가 출력을 만들 때마다 호출될 슬롯 구현
void MainWindow::read_topic_list_output()
{
    // 한 번만 실행되는 경우를 대비해, 첫 출력 시 텍스트를 지웁니다.
    if(ui->topic_list_viewer->toPlainText() == "Loading topic list..."){
        ui->topic_list_viewer->clear();
    }
  // 프로세스의 표준 출력 내용을 읽어와 텍스트 브라우저에 추가
  QByteArray data = topic_process_->readAllStandardOutput();
  ui->topic_list_viewer->append(QString::fromLocal8Bit(data));
}