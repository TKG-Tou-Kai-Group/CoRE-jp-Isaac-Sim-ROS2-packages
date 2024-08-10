import os
import math
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import launch
import xacro

ROBOT_NAME = 'sample_robot_1'
ROBOT_START_POSITION    = [-4.5, 8.5, 0.0]
ROBOT_START_YAW = 0.0

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    flying_disc_description_path = os.path.join(
        get_package_share_directory('flying_disc_description'))

    flying_disc_usd_file = os.path.join(flying_disc_description_path,
                              'meshes', 'USD',
                              'flying_disc_20set.usd')


    sample_robot_description_path = os.path.join(
        get_package_share_directory('sample_robot_description'))

    sample_robot_xacro_file = os.path.join(sample_robot_description_path,
                              'robots',
                              ROBOT_NAME + '.urdf.xacro')
    sample_robot_urdf_path = os.path.join(sample_robot_description_path, 'urdf', ROBOT_NAME + '.urdf')
    # xacroをロード
    sample_robot_doc = xacro.process_file(sample_robot_xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    sample_robot_desc = sample_robot_doc.toprettyxml(indent='  ')
    f = open(sample_robot_urdf_path, 'w')
    f.write(sample_robot_desc)
    f.close()
    sample_robot_relative_urdf_path = pathlib.Path(sample_robot_urdf_path).relative_to(os.getcwd())
    params = {'robot_description': sample_robot_desc}

    rviz_config_file = os.path.join(sample_robot_description_path, 'config', 'sample_robot_description.rviz')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=ROBOT_NAME,
        output='screen',
        parameters=[params]
    )

    isaac_spawn_robot = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        namespace=ROBOT_NAME,
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path),
                    'x' : ROBOT_START_POSITION[0],
                    'y' : ROBOT_START_POSITION[1],
                    'z' : ROBOT_START_POSITION[2],
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : ROBOT_START_YAW,
                    }],
        output='screen',
    )

    isaac_prepare_sensors = Node(
        package="isaac_ros2_scripts",
        executable="prepare_sensors",
        namespace=ROBOT_NAME,
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path)}],
    )

    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        namespace=ROBOT_NAME,
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path)}],
    )

    flying_disc_spawn = Node(
        package="isaac_ros2_scripts",
        executable="add_usd",
        name= "flying_disc_spawn",
        parameters=[{'usd_path': str(flying_disc_usd_file),
                    'usd_name' : ROBOT_NAME + "_flying_disc",
                    'x' : ROBOT_START_POSITION[0],
                    'y' : ROBOT_START_POSITION[1],
                    'z' : ROBOT_START_POSITION[2] + 0.55,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : ROBOT_START_YAW,
                    }],
                output='screen',
                )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("sample_robot_description"),
            "config",
            "sample_robot_config.yaml",
        ]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=ROBOT_NAME,
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/" + ROBOT_NAME + "/controller_manager"],
    )
    
    omni_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller", "--controller-manager", "/" + ROBOT_NAME + "/controller_manager"],
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/" + ROBOT_NAME + "/controller_manager"],
    )

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        namespace=ROBOT_NAME,
        remappings=[
            ('cmd_vel_stamped', 'omni_wheel_controller/cmd_vel'),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=ROBOT_NAME,
        output="log",
        arguments=["-d", rviz_config_file],
    )

    teleop_twist_joy = Node(
            package='teleop_twist_joy_for_sample_robot', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[
                PathJoinSubstitution(
                    [
                        FindPackageShare("sample_robot_description"),
                        "config",
                        "sample_robot_controller.config.yaml",
                    ]
                )
            ],
            namespace=ROBOT_NAME,
            remappings={
                ('cmd_vel', 'cmd_vel'),
                ('commands', 'velocity_controller/commands'),
            },
    )
    
    core_jp_camera_publisher = Node(
        package='core_jp_camera_publisher',
        name='publisher_node',
        executable='publisher_node',
        namespace=ROBOT_NAME,
        remappings=[
            ('input_image_topic', '/World/' + ROBOT_NAME + '/camera_link/image_raw'),
            ('output_image_topic',  '/World/' + ROBOT_NAME + '/camera_link/image_compressed'),
            ('game_status', '/game_status'),
            ('countdown', '/countdown'),
            ('robot1_hp',  '/' + "sample_robot_1" + '/robot_hp'),
            ('robot2_hp',  '/' + "sample_robot_2" + '/robot_hp'),
            ('robot3_hp',  '/' + "sample_robot_3" + '/robot_hp'),
            ('robot4_hp',  '/' + "sample_robot_4" + '/robot_hp'),
            ('robot5_hp',  '/' + "sample_robot_5" + '/robot_hp'),
            ('robot6_hp',  '/' + "sample_robot_6" + '/robot_hp'),
            ('robot7_hp',  '/' + "sample_robot_7" + '/robot_hp'),
            ('robot8_hp',  '/' + "sample_robot_8" + '/robot_hp'),
        ],
    )

    hp_manager = Node(
        package='hp_manager',
        name='hp_manager_node',
        executable='manager_node',
        namespace=ROBOT_NAME,
        remappings=[
            ('armor_topic_1', '/World/' + ROBOT_NAME + '/armor1_link/contact'),
            ('armor_topic_2', '/World/' + ROBOT_NAME + '/armor2_link/contact'),
            ('armor_topic_3', '/World/' + ROBOT_NAME + '/armor3_link/contact'),
            ('armor_topic_4', '/World/' + ROBOT_NAME + '/armor4_link/contact'),
        ],
        parameters=[{
            'initial_hp': 30,
            'respawn_time_sec': 0.0,
        }],
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_spawn_robot,
                on_exit=[isaac_prepare_sensors],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_prepare_sensors,
                on_exit=[isaac_prepare_robot_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=isaac_prepare_robot_controller,
                on_exit=[flying_disc_spawn],
            )
        ),
        node_robot_state_publisher,
        isaac_spawn_robot,
        control_node,
        joint_state_broadcaster_spawner,
        omni_wheel_controller_spawner,
        velocity_controller_spawner,
        velocity_converter,
        #rviz,
        teleop_twist_joy,
        core_jp_camera_publisher,
        hp_manager,
    ]
    )
