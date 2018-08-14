
import os

from ament_index_python.packages import get_package_share_directory
from launch.legacy.exit_handler import restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    process = launch_descriptor

    package = 'cartographer_ros'
    cartographer_prefix = get_package_share_directory('bring_up')
    cartographer_config_dir = os.path.join(cartographer_prefix, 'configuration_files')
    process.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='cartographer_node'),
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'my_cart.lua'
        ],
        name='cartographer_node',
        exit_handler=restart_exit_handler,
    )

    return process
