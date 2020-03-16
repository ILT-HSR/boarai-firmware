import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


MODE_CONTROLLER: ComposableNode = ComposableNode(
    package='boarai_intelligence',
    node_plugin='boarai::intelligence::mode_controller',
    node_name='mode_controller',
    parameters=[]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/intelligence',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            MODE_CONTROLLER,           
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([container])