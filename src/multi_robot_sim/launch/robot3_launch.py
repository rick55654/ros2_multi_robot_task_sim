import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    default_map_path = os.path.join(pkg_share, 'maps', 'map1.png')
    
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map file to use.'
    )

    map_file = LaunchConfiguration('map')

    # 只啟動 robot_agent_3
    robot3_node = Node(
        package='multi_robot_sim',
        executable='robot_agent',
        name='robot_agent_3',
        output='screen',
        parameters=[
            {'map': map_file},
            {'robot_name': 'robot_3'},
            {'init_pose': [100.0, 150.0, -1.57]},
            {'simulator_type': 'basic'},
            {'controller_type': 'pure_pursuit'},
            {'planner_type': 'rrt'}
        ]
    )

    return LaunchDescription([
        map_arg,
        robot3_node,
    ])