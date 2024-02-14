import os
import pathlib

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

import xacro

# This is a time for Isaac Sim complete launching.
WAIT_TIME = 30.0 #seconds

def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    isaac_launcher = Node(
        package="isaac_ros2_scripts",
        executable="launcher",
    )

    core_stage_description_path = os.path.join(
        get_package_share_directory('core_stage_description'))

    core_stage_xacro_file = os.path.join(core_stage_description_path,
                              'urdf',
                              'core_stage.urdf.xacro')
    core_stage_urdf_path = os.path.join(core_stage_description_path, 'urdf', 'core_stage.urdf')
    # xacroをロード
    core_stage_doc = xacro.process_file(core_stage_xacro_file, mappings={'use_sim' : 'true'})
    # xacroを展開してURDFを生成
    core_stage_desc = core_stage_doc.toprettyxml(indent='  ')
    f = open(core_stage_urdf_path, 'w')
    f.write(core_stage_desc)
    f.close()
    core_stage_relative_urdf_path = pathlib.Path(core_stage_urdf_path).relative_to(os.getcwd())

    core_stage_spawn_entity = Node(
        package="isaac_ros2_scripts",
        executable="spawn_robot",
        parameters=[{'urdf_path': str(core_stage_relative_urdf_path),
                    'x' : 0.0,
                    'y' : 0.0,
                    'z' : 0.05,
                    'R' : 0.0,
                    'P' : 0.0,
                    'Y' : 0.0,
                    }],
    )
    
    isaac_spawn_core_stage_timer = TimerAction(period=WAIT_TIME,
        actions=[
            core_stage_spawn_entity,
        ])

    return LaunchDescription([
        isaac_launcher,
        isaac_spawn_core_stage_timer,
    ])
