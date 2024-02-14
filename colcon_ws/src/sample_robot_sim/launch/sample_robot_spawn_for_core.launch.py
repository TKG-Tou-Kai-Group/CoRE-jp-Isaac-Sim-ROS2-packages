import os
import math

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import xacro

ROBOT1_START_POSITION    = [1.0, 2.5, 0.05]
ROBOT1_START_YAW = 0.0
def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])]
             )

    core_stage_description_path = os.path.join(
        get_package_share_directory('core_stage_description'))

    core_stage_xacro_file = os.path.join(core_stage_description_path,
                              'urdf',
                              'core_stage.urdf.xacro')
    # xacroをロード
    core_stage_doc = xacro.process_file(core_stage_xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    core_stage_desc = core_stage_doc.toprettyxml(indent='  ')

    flying_disc_description_path = os.path.join(
        get_package_share_directory('flying_disc_description'))

    flying_disc_xacro_file = os.path.join(flying_disc_description_path,
                              'urdf',
                              'flying_disc.urdf.xacro')

    # xacroをロード
    flying_disc_doc = xacro.process_file(flying_disc_xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    flying_disc_desc = flying_disc_doc.toprettyxml(indent='  ')

    sample_robot_description_path = os.path.join(
        get_package_share_directory('sample_robot_description'))

    xacro_file = os.path.join(sample_robot_description_path,
                              'robots',
                              'sample_robot.urdf.xacro')
    # xacroをロード
    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    robot_desc = doc.toprettyxml(indent='  ')

    params = {'robot_description': robot_desc}

    rviz_config_file = os.path.join(sample_robot_description_path, 'config', 'sample_robot_description.rviz')
    
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-name', 'sample_robot',
                   '-x', str(ROBOT1_START_POSITION[0]),
                   '-y', str(ROBOT1_START_POSITION[1]),
                   '-z', str(ROBOT1_START_POSITION[2]),
                   '-R', str(0.0),
                   '-P', str(0.0),
                   '-Y', str(ROBOT1_START_YAW),
                   '-allow_renaming', 'false'],
    )

    core_stage_gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', core_stage_desc,
                   '-name', 'core_stage',
                   '-x', str(0.0),
                   '-z', LaunchConfiguration('sensor_coord_Z', default=0.05),
                   '-y', LaunchConfiguration('sensor_coord_Y', default=0.0),
                   '-R', LaunchConfiguration('sensor_orien_R', default=0.0),
                   '-P', LaunchConfiguration('sensor_orien_P', default=0.0),
                   '-Y', LaunchConfiguration('sensor_orien_Y', default=0.0),
                   '-allow_renaming', 'true'],
    )

    flying_disc_gz_spawn_entity_list = []
    for index in range(6):
        disc_x = 0.2024*math.cos(-0.47473 + (index+1) * math.pi / 10)
        disc_z = 0.2024*math.sin(-0.47473 + (index+1) * math.pi / 10)
        flying_disc_gz_spawn_entity_list.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=['-string', flying_disc_desc,
                    '-name', 'disc' + str(index),
                    '-x', str(ROBOT1_START_POSITION[0]+disc_x*math.cos(ROBOT1_START_YAW)),
                    '-y', str(ROBOT1_START_POSITION[1]+disc_x*math.sin(ROBOT1_START_YAW)),
                    '-z', str(ROBOT1_START_POSITION[2] + 0.4425+disc_z),
                    '-R', str(0.0),
                    '-P', str(-(index+1) * math.pi / 10),
                    '-Y', str(ROBOT1_START_YAW),
                    '-allow_renaming', 'true'],
            )
        )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_omni_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'omni_wheel_controller'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
    )

    load_velocity_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'velocity_controller'],
        output='screen'
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/front_left/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/rear_right/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                   '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                   '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/depth_camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                   '/depth_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/depth_camera/image_raw/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked'],
        output='screen'
    )

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
                target_action=gz_spawn_entity,
                on_exit=[core_stage_gz_spawn_entity],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_state_controller,
               on_exit=[load_omni_wheel_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_omni_wheel_controller,
               on_exit=[load_joint_trajectory_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=load_joint_trajectory_controller,
               on_exit=[load_velocity_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        gz_spawn_entity,
        bridge,
        velocity_converter,
        rviz,
    ] + flying_disc_gz_spawn_entity_list
    )
