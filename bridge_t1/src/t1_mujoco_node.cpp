/**
 * @file t1_mujoco_node.cpp
 * @brief T1 robot deployment node for MuJoCo simulation
 * 
 * This node connects bridge_core (RL controller) with mujoco_ros (simulator)
 * via ROS2 topics. It subscribes to sensor data from mujoco_ros and publishes
 * control commands back.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/empty.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <mujoco_ros/msg/joint_control_cmd.hpp>

#include <bridge_core/core/types.hpp>
#include <bridge_core/core/config_manager.hpp>
#include <bridge_core/core/rl_controller.hpp>
#include <bridge_core/core/transforms.hpp>
#include <bridge_core/interfaces/robot_interface.hpp>
#include <bridge_core/algorithms/mod.hpp>

#include <mutex>
#include <memory>

using namespace bridge_core;

/**
 * @brief Simulation interface that communicates with mujoco_ros via ROS topics
 */
class MujocoSimInterface : public SimInterface {
public:
    MujocoSimInterface(rclcpp::Node::SharedPtr node)
        : node_(node)
    {
        // Publisher for combined joint control command
        joint_cmd_pub_ = node_->create_publisher<mujoco_ros::msg::JointControlCmd>(
            "/mujoco/joint_cmd", 10);
        
        // Subscribers for sensor data
        joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/mujoco/joint_states", 10,
            std::bind(&MujocoSimInterface::jointStateCallback, this, std::placeholders::_1));
        
        imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
            "/mujoco/imu", 10,
            std::bind(&MujocoSimInterface::imuCallback, this, std::placeholders::_1));
        
        odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
            "/mujoco/odom", 10,
            std::bind(&MujocoSimInterface::odomCallback, this, std::placeholders::_1));
        
        // Service clients
        reset_client_ = node_->create_client<std_srvs::srv::Empty>("/mujoco/reset");
        
        RCLCPP_INFO(node_->get_logger(), "MujocoSimInterface initialized");
    }
    
    void initialize(const RobotConfig& config) override {
        config_ = config;
        
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.resize(static_cast<size_t>(config.num_dof));
        
        // Build name-to-index mapping
        for (size_t i = 0; i < config.joint_names.size(); ++i) {
            joint_name_to_idx_[config.joint_names[i]] = static_cast<int>(i);
        }
        
        is_ready_ = true;
    }
    
    RobotState getState() override {
        std::lock_guard<std::mutex> lock(state_mutex_);
        return state_;
    }
    
    void sendCommand(const RobotCommand& command) override {
        mujoco_ros::msg::JointControlCmd cmd_msg;
        cmd_msg.header.stamp = node_->now();
        
        // Position commands
        cmd_msg.position.resize(command.motor.q.size());
        for (size_t i = 0; i < command.motor.q.size(); ++i) {
            cmd_msg.position[i] = static_cast<double>(command.motor.q[i]);
        }
        
        // Velocity commands
        cmd_msg.velocity.resize(command.motor.dq.size());
        for (size_t i = 0; i < command.motor.dq.size(); ++i) {
            cmd_msg.velocity[i] = static_cast<double>(command.motor.dq[i]);
        }
        
        // Torque commands
        cmd_msg.torque.resize(command.motor.tau.size());
        for (size_t i = 0; i < command.motor.tau.size(); ++i) {
            cmd_msg.torque[i] = static_cast<double>(command.motor.tau[i]);
        }
        
        // Kp gains
        cmd_msg.kp.resize(command.motor.kp.size());
        for (size_t i = 0; i < command.motor.kp.size(); ++i) {
            cmd_msg.kp[i] = static_cast<double>(command.motor.kp[i]);
        }
        
        // Kd gains
        cmd_msg.kd.resize(command.motor.kd.size());
        for (size_t i = 0; i < command.motor.kd.size(); ++i) {
            cmd_msg.kd[i] = static_cast<double>(command.motor.kd[i]);
        }
        
        joint_cmd_pub_->publish(cmd_msg);
    }
    
    bool isReady() const override {
        return is_ready_;
    }
    
    std::string getRobotName() const override {
        return config_.robot_name;
    }
    
    void resetSimulation() override {
        if (reset_client_->wait_for_service(std::chrono::seconds(1))) {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            reset_client_->async_send_request(request);
            RCLCPP_INFO(node_->get_logger(), "Reset request sent");
        } else {
            RCLCPP_WARN(node_->get_logger(), "Reset service not available");
        }
    }
    
    void pauseSimulation(bool pause) override {
        is_paused_ = pause;
        // Note: Could add pause service client if needed
    }
    
    bool isPaused() const override {
        return is_paused_;
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            auto it = joint_name_to_idx_.find(msg->name[i]);
            if (it != joint_name_to_idx_.end()) {
                int idx = it->second;
                if (idx >= 0 && static_cast<size_t>(idx) < state_.motor.q.size()) {
                    if (i < msg->position.size()) {
                        state_.motor.q[idx] = static_cast<float>(msg->position[i]);
                    }
                    if (i < msg->velocity.size()) {
                        state_.motor.dq[idx] = static_cast<float>(msg->velocity[i]);
                    }
                }
            }
        }
    }
    
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        state_.imu.quaternion[0] = static_cast<float>(msg->orientation.w);
        state_.imu.quaternion[1] = static_cast<float>(msg->orientation.x);
        state_.imu.quaternion[2] = static_cast<float>(msg->orientation.y);
        state_.imu.quaternion[3] = static_cast<float>(msg->orientation.z);
        
        state_.imu.gyroscope[0] = static_cast<float>(msg->angular_velocity.x);
        state_.imu.gyroscope[1] = static_cast<float>(msg->angular_velocity.y);
        state_.imu.gyroscope[2] = static_cast<float>(msg->angular_velocity.z);
        
        state_.imu.accelerometer[0] = static_cast<float>(msg->linear_acceleration.x);
        state_.imu.accelerometer[1] = static_cast<float>(msg->linear_acceleration.y);
        state_.imu.accelerometer[2] = static_cast<float>(msg->linear_acceleration.z);
        
        // Compute euler angles from quaternion
        state_.imu.euler = quatToEuler(state_.imu.quaternion);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        
        // Linear velocity in world frame - transform to body frame if needed
        state_.lin_vel_b[0] = static_cast<float>(msg->twist.twist.linear.x);
        state_.lin_vel_b[1] = static_cast<float>(msg->twist.twist.linear.y);
        state_.lin_vel_b[2] = static_cast<float>(msg->twist.twist.linear.z);
    }
    
    rclcpp::Node::SharedPtr node_;
    RobotConfig config_;
    
    // State
    std::mutex state_mutex_;
    RobotState state_;
    std::unordered_map<std::string, int> joint_name_to_idx_;
    bool is_ready_ = false;
    bool is_paused_ = false;
    
    // Publisher for combined control command
    rclcpp::Publisher<mujoco_ros::msg::JointControlCmd>::SharedPtr joint_cmd_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Service clients
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr reset_client_;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<rclcpp::Node>("t1_mujoco_node");
    
    // Get model name (each model has its own directory with config.yaml and model.onnx)
    node->declare_parameter<std::string>("model_name", "t1_mod");
    std::string model_name = node->get_parameter("model_name").as_string();
    
    std::string package_share = ament_index_cpp::get_package_share_directory("bridge_t1");
    std::string config_path = package_share + "/models/" + model_name + "/config.yaml";
    
    RCLCPP_INFO(node->get_logger(), "Loading config from: %s", config_path.c_str());
    
    try {
        // Load configuration
        Config config = ConfigManager::loadConfig(config_path, node);
        
        // Create robot interface
        auto robot_interface = std::make_shared<MujocoSimInterface>(node);
        robot_interface->initialize(config.robot);
        
        // Create algorithm
        auto algorithm = std::make_shared<Mod>();
        algorithm->initialize(node, config.algorithm, config.robot, config.control);
        
        // Create RL controller
        auto controller = std::make_shared<RLController>(
            node, robot_interface, algorithm, config);
        
        // Start controller
        controller->start();
        
        RCLCPP_INFO(node->get_logger(), "T1 MuJoCo node started successfully");
        RCLCPP_INFO(node->get_logger(), "  Robot: %s", config.robot.robot_name.c_str());
        RCLCPP_INFO(node->get_logger(), "  Algorithm: %s", config.algorithm.name.c_str());
        RCLCPP_INFO(node->get_logger(), "  DOFs: %d (active: %zu)", 
                    config.robot.num_dof, config.robot.dof_activated.size());
        
        rclcpp::spin(node);
        
        controller->stop();
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
