import os
import math
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

ROBOT1_START_POSITION    = [3.0, 5.5, 1.05]
ROBOT1_START_YAW = 0.0
def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    flying_disc_description_path = os.path.join(
        get_package_share_directory('flying_disc_description'))

    flying_disc_xacro_file = os.path.join(flying_disc_description_path,
                              'urdf',
                              'flying_disc.urdf.xacro')
    flying_disc_urdf_path = os.path.join(flying_disc_description_path, 'urdf', 'flying_disc.urdf')
    # xacroをロード
    flying_disc_doc = xacro.process_file(flying_disc_xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    flying_disc_desc = flying_disc_doc.toprettyxml(indent='  ')
    f = open(flying_disc_urdf_path, 'w')
    f.write(flying_disc_desc)
    f.close()
    flying_disc_relative_urdf_path = pathlib.Path(flying_disc_urdf_path).relative_to(os.getcwd())


    sample_robot_description_path = os.path.join(
        get_package_share_directory('sample_robot_description'))

    sample_robot_xacro_file = os.path.join(sample_robot_description_path,
                              'robots',
                              'sample_robot.urdf.xacro')
    sample_robot_urdf_path = os.path.join(sample_robot_description_path, 'urdf', 'sample_robot.urdf')
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
        output='screen',
        parameters=[params]
    )

    isaac_spawn_robot = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path),
                    'x' : ROBOT1_START_POSITION[0],
                    'y' : ROBOT1_START_POSITION[1],
                    'z' : ROBOT1_START_POSITION[2],
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : ROBOT1_START_YAW,
                    }],
        output='screen',
    )

    isaac_prepare_sensors = Node(
        package="isaac_ros2_scripts",
        executable="prepare_sensors",
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path)}],
    )

    isaac_prepare_robot_controller = Node(
        package="isaac_ros2_scripts",
        executable="prepare_robot_controller",
        parameters=[{'urdf_path': str(sample_robot_relative_urdf_path)}],
    )

    flying_disc_gz_spawn_entity_list = []
    for index in range(6):
        disc_x = 0.2024*math.cos(-0.47473 + (index+1) * math.pi / 10)
        disc_z = 0.2024*math.sin(-0.47473 + (index+1) * math.pi / 10)
        flying_disc_gz_spawn_entity_list.append(
            Node(
                package="isaac_ros2_scripts",
                executable="spawn_robot",
                parameters=[{'urdf_path': str(flying_disc_relative_urdf_path),
                            'x' : ROBOT1_START_POSITION[0]+disc_x*math.cos(ROBOT1_START_YAW),
                            'y' : ROBOT1_START_POSITION[1]+disc_x*math.sin(ROBOT1_START_YAW),
                            'z' : ROBOT1_START_POSITION[2] + 0.4425+disc_z,
                            'R' : 0.0,
                            'P' : -(index+1) * math.pi / 10,
                            'Y' : ROBOT1_START_YAW,
                            }],
                output='screen',
                )
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
        parameters=[params, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    omni_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_controller", "--controller-manager", "/controller_manager"],
    )

    #load_joint_trajectory_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'joint_trajectory_controller'],
    #    output='screen'
    #)

    #load_velocity_controller = ExecuteProcess(
    #    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'velocity_controller'],
    #    output='screen'
    #)

    velocity_converter = Node(
        package='velocity_pub',
        name='velocity_pub',
        executable='velocity_pub',
        remappings=[
            ('/cmd_vel_stamped', '/omni_wheel_controller/cmd_vel'),
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
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
        node_robot_state_publisher,
        isaac_spawn_robot,
        control_node,
        joint_state_broadcaster_spawner,
        omni_wheel_controller_spawner,
        velocity_converter,
        rviz,
    ] #+ flying_disc_gz_spawn_entity_list
    )
