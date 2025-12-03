cmake_minimum_required(VERSION 3.14)
project(bridge_t1)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(bridge_core REQUIRED)
find_package(mujoco_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Create T1 MuJoCo node
add_executable(t1_mujoco_node
    src/t1_mujoco_node.cpp
)

target_include_directories(t1_mujoco_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(t1_mujoco_node
    rclcpp
    std_msgs
    sensor_msgs
    nav_msgs
    std_srvs
    bridge_core
    mujoco_ros
    ament_index_cpp
)

# Install executable
install(TARGETS t1_mujoco_node
    DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# Install models (each model has config.yaml and model.onnx)
install(DIRECTORY models
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
