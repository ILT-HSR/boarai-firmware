import launch

from launch_ros.actions import ComposableNodeContainer, Node
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

    gamepad = Node(
        node_name='gamepad',
        node_namespace='/boarai/interface',
        package='joystick_ros2',
        node_executable='joystick_ros2',
        output='screen',
        emulate_tty=True,
        remappings=[
            (
                "rostopic://joy",
                "gamepad"
            ),
        ]
    )

    return launch.LaunchDescription([container, gamepad])