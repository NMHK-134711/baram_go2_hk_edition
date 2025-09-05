# baram_go2_client_driver
**Unitree Go2 driver for BARAM GO2 with ROS2 Humble**

# Dependencies
1. `unitree_ros2`
   - It contains the ROS Interfaces to communicate Unitree Robots and examples.
   - If you want to know how develop those classes, modules, and methods to call apis, follow the examples in `unitree_ros2` package.
2. `unitree_sdk2`
   - It contains `unitree_go`, `unitree_api`
3. `baram_go2_interfaces`
4. `nlohmann_json`

# Clone & Build
1. Configure [`unitree_ros2`](https://github.com/unitreerobotics/unitree_ros2.git) environment.
   - Please follow `unitree_ros2` README to configure.
     - For example, it uses `cyclonedds`.
2. Install [`unitree_sdk2`](https://github.com/unitreerobotics/unitree_sdk2.git)
   - Follow `unitree_sdk2` to build and install binary package.
3. (_**IMPORTANT**_) Clone this repository into **`unitree_ros2`** workspace.
   - Clone into the following directory.
      ```
      unitree_ros2/
      -> cyclonedds_ws/
        -> src/ (**clone here**)
      ```
      ```
      git clone https://github.com/ggonu/baram_go2_client_driver.git
      ```
4. Build
   1. Build `cyclonedds` first.
      - Build at `unitree_ros2/cyclonedds_ws/`.
      ```(bash)
      colcon build --packages-select cyclonedds
      ```
   2. Source ROS2 in the terminal
      ```(bash)
      source /opt/ros/humble/setup.bash
      ```
   3. Build the `cyclonedds_ws`
      - Build at `unitree_ros2/cyclonedds_ws/`.
      ```(bash)
      colcon build
      ```
5. Sourcing
   - Go to root directory of `unitree_ros2/`
      ```(bash)
      . setup.sh
      ```

# Usage
1. Run go2_client_driver node only
    ```(bash)
    ros2 run baram_go2_client_driver go2_client_driver
    ```
2. Launch joy control node only
    ```(bash)
    ros2 launch baram_go2_client_driver joy.launch.py
    ```
3. Launch together
    ```(bash)
    ros2 launch baram_go2_client_driver baram_go2_driver.launch.py
    ```

## Nodes
- `go2_client_driver`
  - For unitree_api calls, based on `go2_api.cpp`.

## Launch
- `baram_go2_client_driver.launch.py`
- `joy.launch.py`