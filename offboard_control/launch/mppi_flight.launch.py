from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node=Node(package='offboard_control',executable='mppi_flight',
              name='mppi_sitl_flight',output='screen',
              arguments=['--output',LaunchConfiguration('output')])
    return LaunchDescription([
        DeclareLaunchArgument('output',description='Unique flight evidence directory'),node,
        RegisterEventHandler(OnProcessExit(target_action=node,on_exit=[EmitEvent(event=Shutdown())])),
    ])
