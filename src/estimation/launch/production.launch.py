import launch

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


POSITION_ESTIMATOR: ComposableNode = ComposableNode(
    package='boarai_estimation',
    node_plugin='boarai::estimation::position_estimator',
    node_name='cortex',
    parameters=[]
)


def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/estimation',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            POSITION_ESTIMATOR,           
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([container])