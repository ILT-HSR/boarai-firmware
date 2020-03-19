import launch

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


HMI: ComposableNode = ComposableNode(
    package='boarai_interface',
    node_plugin='boarai::interface::hmi',
    node_name='hmi',
    parameters=[]
)

GAMEPAD: ComposableNode = ComposableNode(
    package='boarai_interface',
    node_plugin='boarai::interface::gamepad',
    node_name='gamepad',
    parameters=[]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/interface',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            HMI,           
            GAMEPAD,
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        container,
    ])