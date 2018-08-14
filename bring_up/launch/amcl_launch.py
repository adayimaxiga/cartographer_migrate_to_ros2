
import os

from ament_index_python.packages import get_package_share_directory
from launch.legacy.exit_handler import restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    process = launch_descriptor

#    package = 'map_server'
#    process.add_process(
#        cmd=[
#            get_executable_path(package_name=package, executable_name='map_server'),
#            '/home/droid/ros2_ws/src/tools/mymap.yaml'
#        ],
#        name='map_server',
#        exit_handler=restart_exit_handler,
#    )

    package = 'amcl'
    process.add_process(
        cmd=[
            get_executable_path(package_name=package, executable_name='amcl'),'--use-map-topic'
        ],
        name='amcl',
        exit_handler=restart_exit_handler,
    )

    return process
