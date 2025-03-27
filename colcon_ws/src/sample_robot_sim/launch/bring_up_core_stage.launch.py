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
                              'core_2025_stage.usd')

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher_with_reset",
        parameters=[{'usd_path': str(core_stage_usd_path),
                     'fps': 10.0,
                     'time_steps_per_second': 120.0,
                     }],
    )

    game_manager = Node(
        package="game_manager",
        executable="manager_node",
        parameters=[{'initial_time': 120,
                     }],
        remappings=[
            ('robot1_hp', '/sample_robot_1/robot_hp'),
            ('robot2_hp', '/sample_robot_2/robot_hp'),
            ('robot3_hp', '/sample_robot_3/robot_hp'),
            ('robot4_hp', '/sample_robot_4/robot_hp'),
            ('robot5_hp', '/sample_robot_5/robot_hp'),
            ('robot6_hp', '/sample_robot_6/robot_hp'),
            ('robot7_hp', '/sample_robot_7/robot_hp'),
            ('robot8_hp', '/sample_robot_8/robot_hp'),
        ],
    )
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('rosbridge_server'), '/launch/rosbridge_websocket_launch.xml'
        ])
    )

    return LaunchDescription([
        isaac_launcher,
        game_manager,
        rosbridge_launch,
    ])
