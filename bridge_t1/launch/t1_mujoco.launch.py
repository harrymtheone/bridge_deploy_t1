"""
Launch file for T1 robot in MuJoCo simulation

This launch file starts:
1. mujoco_ros - MuJoCo simulator with T1 model
2. bridge_t1 - RL controller for T1 robot  
3. robot_state_publisher - For RViz visualization
4. (Optional) RViz for visualization
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='t1_mod',
        description='Name of the model directory (contains config.yaml and model.onnx)'
    )
    
    mjcf_model_arg = DeclareLaunchArgument(
        'mjcf_model',
        default_value='T1_serial_cyliner.xml',
        description='MJCF model file to use (T1_torque.xml or T1_serial_cyliner.xml)'
    )
    
    sim_rate_arg = DeclareLaunchArgument(
        'sim_rate',
        default_value='1000.0',
        description='Simulation rate in Hz'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='500.0',
        description='Publish rate for sensor data in Hz'
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run without MuJoCo GUI visualization'
    )
    
    # Get package share directories
    t1_description_share = FindPackageShare('t1_description')
    
    # Model path for MJCF (configurable)
    model_path = PathJoinSubstitution([
        t1_description_share, 'mjcf', LaunchConfiguration('mjcf_model')
    ])
    
    # Joint names for T1
    joint_names = [
        'AAHead_yaw', 'Head_pitch',
        'Left_Shoulder_Pitch', 'Left_Shoulder_Roll', 'Left_Elbow_Pitch', 'Left_Elbow_Yaw',
        'Right_Shoulder_Pitch', 'Right_Shoulder_Roll', 'Right_Elbow_Pitch', 'Right_Elbow_Yaw',
        'Waist',
        'Left_Hip_Pitch', 'Left_Hip_Roll', 'Left_Hip_Yaw', 'Left_Knee_Pitch', 'Left_Ankle_Pitch', 'Left_Ankle_Roll',
        'Right_Hip_Pitch', 'Right_Hip_Roll', 'Right_Hip_Yaw', 'Right_Knee_Pitch', 'Right_Ankle_Pitch', 'Right_Ankle_Roll'
    ]
    
    # MuJoCo simulation node (runs physics and publishes to ROS)
    mujoco_node = Node(
        package='mujoco_ros',
        executable='mujoco_sim_node',
        name='mujoco_sim',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'robot_name': 'T1',
            'sim_rate': LaunchConfiguration('sim_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'joint_names': joint_names,
            'base_link_name': 'Trunk',
            'headless': LaunchConfiguration('headless')
        }],
        # Remap joint_states for robot_state_publisher
        remappings=[
            ('/mujoco/joint_states', '/joint_states')
        ]
    )
    
    # T1 controller node
    t1_node = Node(
        package='bridge_t1',
        executable='t1_mujoco_node',
        name='t1_controller',
        output='screen',
        parameters=[{
            'model_name': LaunchConfiguration('model_name')
        }]
    )
    
    # RViz (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    # Joystick node for control input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.1,
            'autorepeat_rate': 20.0
        }]
    )
    
    return LaunchDescription([
        model_name_arg,
        mjcf_model_arg,
        sim_rate_arg,
        publish_rate_arg,
        rviz_arg,
        headless_arg,
        mujoco_node,
        joy_node,
        rviz_node,
        t1_node,
    ])
