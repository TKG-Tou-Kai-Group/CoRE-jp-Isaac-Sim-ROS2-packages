import os
import signal
import rclpy
from rclpy.node import Node
import subprocess
from ament_index_python.packages import get_package_share_directory

class SimLancher(Node):
    def __init__(self):
        super().__init__('sim_launcher')

        self.declare_parameter('usd_path', '')
        usd_path = self.get_parameter('usd_path').get_parameter_value().string_value
        if usd_path == '':
            usd_path = os.path.join(
                get_package_share_directory('isaac_ros2_scripts'), 'meshes/USD/default_stage.usd')

        self.proc = None
        
        python_script = os.path.join(
                    '/isaac-sim', 'python.sh')
        start_script = os.path.join(
                    get_package_share_directory('isaac_ros2_scripts'), 'start_sim.py')
        command = ["bash", python_script, start_script, usd_path]
        print(command)
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"]=os.path.join(
                get_package_share_directory('isaac_ros2_scripts'), 'config/fastdds.xml')
        self.proc = subprocess.Popen(command, preexec_fn=os.setsid)        
    
    def __del__(self):
        if not self.proc == None:
            if self.proc.poll() is None:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = SimLancher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
