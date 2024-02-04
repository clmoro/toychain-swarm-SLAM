import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnProcessStart)
from launch.events import Shutdown

 
def generate_launch_description():
  nc1 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_1',
      output='screen')
  ne1 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_1',
      output='screen')
  nc2 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_2',
      output='screen')
  ne2 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_2',
      output='screen')
  nc3 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_3',
      output='screen')
  ne3 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_3',
      output='screen')
  nc4 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_4',
      output='screen')
  ne4 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_4',
      output='screen')
  nc5 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_5',
      output='screen')
  ne5 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_5',
      output='screen')
  nc6 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_6',
      output='screen')
  ne6 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_6',
      output='screen')
  nc7 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_7',
      output='screen')
  ne7 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_7',
      output='screen')
  nc8 = Node(package='multiturtlebots_controller_pkg', executable='robot_controller_8',
      output='screen')
  ne8 = Node(package='multiturtlebots_controller_pkg', executable='robot_estimator_8',
      output='screen')
  no1 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_1',
      output='screen')
  no2 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_2',
      output='screen')
  no3 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_3',
      output='screen')
  no4 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_4',
      output='screen')
  no5 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_5',
      output='screen')
  no6 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_6',
      output='screen')
  no7 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_7',
      output='screen')
  no8 = Node(package='multiturtlebots_controller_pkg', executable='odometry_noise_8',
      output='screen')
  start_register_bag = launch.actions.ExecuteProcess(cmd=['ros2', 'bag', 'record', '-o', 'odometry_bag', '/bot1/odom', '/bot1/noisy_odom', '/bot2/odom', '/bot2/noisy_odom', '/bot3/odom', '/bot3/noisy_odom', '/bot4/odom', '/bot4/noisy_odom', '/bot5/odom', '/bot5/noisy_odom', '/bot6/odom', '/bot6/noisy_odom', '/bot7/odom', '/bot7/noisy_odom', '/bot8/odom', '/bot8/noisy_odom', '/blockchain_transformation_C0', '/blockchain_transformation_C1', '/blockchain_transformation_C2', '/blockchain_transformation_C3', '/blockchain_transformation_C4', '/blockchain_transformation_C5', '/blockchain_transformation_R0', '/blockchain_transformation_R1', '/blockchain_transformation_R2', '/blockchain_transformation_R3', '/blockchain_transformation_R4', '/blockchain_transformation_R5', '/blockchain_transformation_C0t', '/blockchain_transformation_C1t', '/blockchain_transformation_C2t', '/blockchain_transformation_C3t', '/blockchain_transformation_C4t', '/blockchain_transformation_C5t'], 
      output='screen')
 
  return LaunchDescription([
    nc1,
    nc2,
    nc3,
    nc4,
    nc5,
    nc6,
    nc7,
    nc8,
    ne1,
    ne2,
    ne3,
    ne4,
    ne5,
    ne6,
    ne7,
    ne8,
    no1,
    no2,
    no3,
    no4,
    no5,
    no6,
    no7,
    no8,
    RegisterEventHandler(
            OnProcessStart(
                target_action=no8,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[start_register_bag],
                    )
                ]
            )
    ),
    RegisterEventHandler(
            OnProcessStart(
                target_action=start_register_bag,
                on_start=[
                    TimerAction(
                        period=2400.0,
                        actions=[EmitEvent(event=Shutdown(reason='Registration finished'))],
                    )
                ]
            )
    ),
  ])
