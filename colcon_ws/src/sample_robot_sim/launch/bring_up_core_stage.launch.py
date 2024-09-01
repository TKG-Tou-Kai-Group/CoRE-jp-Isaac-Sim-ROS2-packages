import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)


    core_stage_description_path = os.path.join(
        get_package_share_directory('core_stage_description'))

    core_stage_usd_path = os.path.join(core_stage_description_path,
                              'meshes', 'USD',
                              'core_stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
        parameters=[{'usd_path': str(core_stage_usd_path),
                     'fps': 10.0,
                     'time_steps_per_second': 120.0,
                     }],
    )
    
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('rosbridge_server'), '/launch/rosbridge_websocket_launch.xml'
        ])
    )

    return LaunchDescription([
        isaac_launcher,
        rosbridge_launch,
    ])
