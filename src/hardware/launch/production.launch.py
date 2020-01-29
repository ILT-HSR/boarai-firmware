import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


MOTOR_CONTROL: ComposableNode = ComposableNode(
    package='boarai_hardware',
    node_plugin='boarai::hardware::motor_control',
    node_name='motor_control',
    parameters=[{
        "driver_enabled": True,
        "driver_address": "192.168.1.20",
        "driver_port": 502,
    }]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/hardware',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            MOTOR_CONTROL,           
        ],
        output='screen'
    )

    return launch.LaunchDescription([container])