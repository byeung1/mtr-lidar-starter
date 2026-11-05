import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('vrx_nav2')
    params = os.path.join(share, 'config', 'nav2.yaml')
    nodes = ['map_server', 'planner_server', 'controller_server', 'bt_navigator', 'waypoint_follower']
    actions = [DeclareLaunchArgument('start_sim', default_value='true'),
               DeclareLaunchArgument('initially_armed', default_value='false'),
               DeclareLaunchArgument('camera_rate', default_value='2.0'),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'interfaces.launch.py')),
                   launch_arguments={'start_sim': LaunchConfiguration('start_sim'),
                                     'initially_armed': LaunchConfiguration('initially_armed'),
                                     'camera_rate': LaunchConfiguration('camera_rate')}.items())]
    for name in nodes:
        package = {'map_server': 'nav2_map_server', 'planner_server': 'nav2_planner',
                   'controller_server': 'nav2_controller', 'bt_navigator': 'nav2_bt_navigator',
                   'waypoint_follower': 'nav2_waypoint_follower'}[name]
        overrides = {'use_sim_time': True}
        if name == 'map_server':
            overrides['yaml_filename'] = os.path.join(share, 'config', 'water.yaml')
        if name == 'bt_navigator':
            overrides['default_nav_to_pose_bt_xml'] = os.path.join(share, 'behavior_trees', 'to_pose.xml')
            overrides['default_nav_through_poses_bt_xml'] = os.path.join(share, 'behavior_trees', 'through_poses.xml')
        actions.append(Node(package=package, executable=name, name=name, output='screen',
            parameters=[params, overrides], remappings=[('cmd_vel', '/nav/cmd_vel')]))
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_boat', output='screen', parameters=[{
            'use_sim_time': True, 'autostart': True, 'node_names': nodes,
            'bond_timeout': 10.0}]))
    return LaunchDescription(actions)
