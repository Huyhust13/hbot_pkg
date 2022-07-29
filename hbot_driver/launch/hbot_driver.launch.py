import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('hbot_driver'),
        'config',
        'params.yaml'
        )

    node=Node(
        package = 'hbot_driver',
        name = 'hbot_driver_node',
        executable = 'hbot_driver',
        parameters = [config]
    )

    # node=Node(
    #     package = 'hbot_driver',
    #     name = 'hbot_driver_node',
    #     executable = 'hbot_driver',
    #     parameters = [
    #         {"baud_rate": 115200},
    #         {"port": '/dev/ttyUSB1'},
    #         {"kp": 10.0},
    #         {"ki": 0.0},
    #         {"kd": 10.0},
    #         ]
    # )

    ld.add_action(node)
    return ld