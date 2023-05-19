import os
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
    Node(package='multiturtlebots_controller_pkg', executable='odometry_noise',
      output='screen'),
  ])
