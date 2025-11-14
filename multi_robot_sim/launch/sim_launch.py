import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_share = get_package_share_directory('multi_robot_sim')
    default_map_path = os.path.join(pkg_share, 'maps', 'map1.png')
    
    # 每個檔案都需要地圖參數，確保大家用的是同一張地圖
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to the map file to use.'
    )

    map_file = LaunchConfiguration('map')

    # 只啟動 simulation_node
    sim_node = Node(
        package='multi_robot_sim',
        executable='simulation_node',
        name='simulation_node',
        output='screen',
        parameters=[{'map': map_file}]
    )

    return LaunchDescription([
        map_arg,
        sim_node,
    ])