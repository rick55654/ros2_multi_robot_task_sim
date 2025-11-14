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

    # 只啟動 robot_agent_2
    robot2_node = Node(
        package='multi_robot_sim',
        executable='robot_agent',
        name='robot_agent_2',
        output='screen',
        parameters=[
            {'map': map_file},
            {'robot_name': 'robot_2'},
            {'init_pose': [150.0, 200.0, 1.57]},
            {'simulator_type': 'bicycle'},
            {'controller_type': 'stanley'},
            {'planner_type': 'rrt_star'}
        ]
    )

    return LaunchDescription([
        map_arg,
        robot2_node,
    ])