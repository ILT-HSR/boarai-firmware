import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


HMI: ComposableNode = ComposableNode(
    package='boarai_interface',
    node_plugin='boarai::interface::hmi',
    node_name='hmi',
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
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([container])