
import os

from ament_index_python.packages import get_package_share_directory
from launch.legacy.exit_handler import restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    process = launch_descriptor
    
#    package = 'tf2_ros'
#    process.add_process(
#        cmd=[
#            get_executable_path(
#                package_name=package, executable_name='static_transform_publisher'),
#            '0', '0', '0',
#            '0', '0', '1', '0',
#            'base_link',
#            'laser'
#        ],
#        name='tf_pub_base_to_laserA',
#        exit_handler=restart_exit_handler,
#    )
    package = 'tf2_ros'
    process.add_process(
        cmd=[
            get_executable_path(
                package_name=package, executable_name='static_transform_publisher'),
            '0.505', '0.259', '0.0',
            '0', '0', '0.382683', '0.92388',
            'base_link',
            'laser'
        ],
        name='tf_pub_base_to_laser_1',
        exit_handler=restart_exit_handler,
    )

    package = 'tf2_ros'
    process.add_process(
        cmd=[
            get_executable_path(
                package_name=package, executable_name='static_transform_publisher'),
            '-0.505', '-0.259', '0.0',
            '0', '0', '-0.92388', '0.382683',
            'base_link',
            'laser_1'
        ],
        name='tf_pub_base_to_laser_2',
        exit_handler=restart_exit_handler,
    )

    package = 'cartographer_ros'
   # cartographer_prefix = '/home/adayimaxiga/mine_ws/src/bring_up'
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
