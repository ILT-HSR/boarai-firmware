import launch

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


GPS_PROVIDER: ComposableNode = ComposableNode(
    package='boarai_hardware',
    node_plugin='boarai::hardware::gps_provider',
    node_name='gps_provider',
    parameters=[]
)

TANK_DRIVE: ComposableNode = ComposableNode(
    package='boarai_hardware',
    node_plugin='boarai::hardware::tank_drive',
    node_name='tank_drive',
    parameters=[{
        "driver_enabled": True,
        "driver_address": "192.168.1.20",
        "driver_port": 502,
        "wheel_spacing": 0.51,
        "maximum_linear_velocity": 2.9,
    }]
)

GAMEPAD: Node = Node(
    node_name='gamepad',
    node_namespace='/boarai/hardware',
    package='joystick_ros2',
    node_executable='joystick_ros2',
    output='screen',
    emulate_tty=True,
    parameters=[{
        "deadzone": 0.1,
    }],
    remappings=[
        (
            "rostopic://joy",
            "gamepad"
        ),
    ]
)

def generate_launch_description() -> launch.LaunchDescription:
    container = ComposableNodeContainer(
        node_name='container',
        node_namespace='/boarai/hardware',
        package='rclcpp_components',
        node_executable='component_container',
        composable_node_descriptions=[
            GPS_PROVIDER,
            TANK_DRIVE,
        ],
        output='screen',
        emulate_tty=True,
    )

    return launch.LaunchDescription([
        container,
        GAMEPAD
    ])