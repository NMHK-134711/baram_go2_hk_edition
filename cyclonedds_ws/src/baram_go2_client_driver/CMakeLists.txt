cmake_minimum_required(VERSION 3.8)
project(baram_go2_client_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(baram_go2_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(baram_go2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
# ==================== Qt5 패키지 찾기 및 설정 ====================
find_package(Qt5 REQUIRED Widgets)
# =============================================================

add_executable(go2_client_driver src/go2_client_driver.cpp src/go2_api.cpp src/motor_crc.cpp)
target_include_directories(go2_client_driver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_client_driver PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_client_driver
  "rclcpp" "sensor_msgs" "geometry_msgs" "nav_msgs" "unitree_go"
  "unitree_api" "tf2_ros" "baram_go2_interfaces" "nlohmann_json"
)

add_executable(go2_robot_state src/go2_robot_state.cpp)
target_include_directories(go2_robot_state PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_robot_state PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_robot_state
  "rclcpp" "sensor_msgs" "geometry_msgs" "unitree_go" "unitree_api"
  "tf2_ros" "baram_go2_interfaces" "nlohmann_json"
)

add_executable(go2_main_node src/go2_main_node.cpp)
target_include_directories(go2_main_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(go2_main_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  go2_main_node
  "rclcpp" "baram_go2_interfaces" "std_msgs" "unitree_go"
)

# ==================== GUI 노드 실행 파일 추가 (완전 수동 방식) ====================

# 1. MOC를 실행할 헤더 파일을 직접 지정합니다.
qt5_wrap_cpp(MOC_SOURCES include/baram_go2_client_driver/gui_main_window.hpp)

# 2. UI 파일을 C++ 헤더로 변환합니다.
qt5_wrap_ui(UI_HEADERS ui/data_browser.ui)

# 3. 원본 소스, UI 변환 헤더, MOC 생성 소스를 모두 포함하여 실행 파일을 생성합니다.
add_executable(gui_node
  src/gui_main.cpp
  src/gui_main_window.cpp
  ${UI_HEADERS}
  ${MOC_SOURCES}
)

target_include_directories(gui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_BINARY_DIR}
)
ament_target_dependencies(gui_node
  "rclcpp"
  "unitree_go"
  "sensor_msgs"
  "unitree_api"
  "baram_go2_interfaces"
)
target_link_libraries(gui_node Qt5::Widgets)
# ========================================================================

install(TARGETS
  go2_client_driver
  go2_robot_state
  go2_main_node
  gui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()