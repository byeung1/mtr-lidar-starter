from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bag = LaunchConfiguration('bag')
    return LaunchDescription([
        DeclareLaunchArgument('bag', default_value='bags/obstacle_case'),
        DeclareLaunchArgument('play_bag', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(package='mtr_lidar_starter', executable='lidar_node', output='screen',
             parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        TimerAction(period=2.0, actions=[ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag, '--clock', '--loop', '--read-ahead-queue-size', '30'],
            output='screen', condition=IfCondition(LaunchConfiguration('play_bag')))]),
    ])
