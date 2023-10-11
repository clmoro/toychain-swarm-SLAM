import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
  return LaunchDescription([
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_1',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_1',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_2',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_2',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_3',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_3',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_4',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_4',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_5',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_5',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_6',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_6',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_7',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_7',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_controller_8',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_8',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_1',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_2',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_3',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_4',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_5',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_6',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_7',
      output='screen'),
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_8',
      output='screen'),
    launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '-b', '500000000', '-o', 'odometry_bag', '/bot1/odom', '/bot1/noisy_odom', '/bot2/odom', '/bot2/noisy_odom', '/bot3/odom', '/bot3/noisy_odom', '/bot4/odom', '/bot4/noisy_odom', '/bot5/odom', '/bot5/noisy_odom', '/bot6/odom', '/bot6/noisy_odom', '/bot7/odom', '/bot7/noisy_odom', '/bot8/odom', '/bot8/noisy_odom'], 
      output='screen'),
  ])
