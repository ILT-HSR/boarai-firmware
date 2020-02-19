import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


POSITION_CONTROL: ComposableNode = ComposableNode(
    package='boarai_control',
    node_plugin='boarai::control::position_control',
    node_name='position_control',
    parameters=[]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/control',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            POSITION_CONTROL,           
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([container])