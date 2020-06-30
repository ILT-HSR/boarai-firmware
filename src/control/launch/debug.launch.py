import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


VELOCITY_CONTROL: ComposableNode = ComposableNode(
    package='boarai_control',
    node_plugin='boarai::control::velocity_control',
    node_name='velocity_control',
    parameters=[]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/control',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            VELOCITY_CONTROL,           
        ],
        output='screen',
        emulate_tty=True,
        prefix='gdb -ex=r --args'
    )

    return launch.LaunchDescription([container])