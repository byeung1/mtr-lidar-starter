import os
from tempfile import TemporaryDirectory
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
                            OpaqueFunction, SetLaunchConfiguration, RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from vrx_nav2.robot_profile import prepare_robot


def prepare_profile(context):
    if LaunchConfiguration('start_sim').perform(context).lower() != 'true':
        return []
    directory = TemporaryDirectory(prefix='vrx-nav2-model-')
    try:
        robot = prepare_robot(directory.name, float(LaunchConfiguration('camera_rate').perform(context)))
    except Exception:
        directory.cleanup()
        raise
    def cleanup(context):
        directory.cleanup()
        return []
    return [SetLaunchConfiguration('navigation_robot', robot),
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)]))]


def generate_launch_description():
    topic = '/world/sydney_regatta/model/wamv/link/wamv/base_link/sensor/lidar_wamv_sensor/scan/points'
    return LaunchDescription([
        DeclareLaunchArgument('start_sim', default_value='true'),
        DeclareLaunchArgument('initially_armed', default_value='false'),
        DeclareLaunchArgument('camera_rate', default_value='2.0',
                             description='Camera Hz for this local Nav2 model (LiDAR unchanged); use 30 for stock rate'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'd3d12'),
        SetEnvironmentVariable('MESA_D3D12_DEFAULT_ADAPTER_NAME', 'NVIDIA'),
        OpaqueFunction(function=prepare_profile),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vrx_gz'), 'launch', 'competition.launch.py')),
            condition=IfCondition(LaunchConfiguration('start_sim')),
            launch_arguments={'world': 'sydney_regatta', 'headless': 'true',
                              'bridge_competition_topics': 'false',
                              'urdf': LaunchConfiguration('navigation_robot', default='')}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='nav_lidar_bridge',
             arguments=[topic+'@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                        '/world/sydney_regatta/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             remappings=[(topic, '/lidar/points'), ('/world/sydney_regatta/clock', '/clock')],
             parameters=[{'use_sim_time': True}]),
        Node(package='vrx_nav2', executable='ground_truth', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='vrx_nav2', executable='cloud_filter', parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='vrx_nav2', executable='thrust_adapter', output='screen', parameters=[{
            'use_sim_time': True, 'initially_armed': LaunchConfiguration('initially_armed')}]),
    ])
