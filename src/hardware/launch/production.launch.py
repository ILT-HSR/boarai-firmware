import launch
from launch.conditions.if_condition import IfCondition
from launch.conditions.unless_condition import UnlessCondition
from launch.substitutions.launch_configuration import LaunchConfiguration

from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


GPS_PROVIDER: ComposableNode = ComposableNode(
    package="boarai_hardware",
    node_plugin="boarai::hardware::gps_provider",
    node_name="gps_provider",
    parameters=[],
)

IMU: ComposableNode = ComposableNode(
    package="boarai_hardware",
    node_plugin="boarai::hardware::imu",
    node_name="imu",
    parameters=[],
)

TANK_DRIVE: ComposableNode = ComposableNode(
    package="boarai_hardware",
    node_plugin="boarai::hardware::tank_drive",
    node_name="tank_drive",
    parameters=[
        {
            "driver_enabled": True,
            "driver_address": "192.168.1.20",
            "driver_port": 502,
            "wheel_spacing": 0.51,
            "maximum_linear_velocity": 2.9,
        }
    ],
)

TANK_DRIVE_FAKE: ComposableNode = ComposableNode(
    package="boarai_hardware",
    node_plugin="boarai::hardware::tank_drive_debug",
    node_name="tank_drive_debug",
    parameters=[
        {
            "wheel_spacing": 0.51,
            "maximum_linear_velocity": 2.9,
            "acceleration_delay": 0.1,
        }
    ],
)

GAMEPAD: Node = Node(
    node_name="gamepad",
    node_namespace="/boarai/hardware",
    package="joystick_ros2",
    node_executable="joystick_ros2",
    output="screen",
    emulate_tty=True,
    parameters=[
        {
            "deadzone": 0.1,
        }
    ],
    remappings=[
        ("rostopic://joy", "gamepad"),
    ],
)


def generate_launch_description() -> launch.LaunchDescription:
    fake_tank_drive = LaunchConfiguration("fake_tank_drive")

    return launch.LaunchDescription(
        [
            ComposableNodeContainer(
                node_name="container",
                node_namespace="/boarai/hardware",
                package="rclcpp_components",
                node_executable="component_container_mt",
                composable_node_descriptions=[
                    GPS_PROVIDER,
                    TANK_DRIVE_FAKE,
                    IMU,
                ],
                output="screen",
                emulate_tty=True,
                condition=IfCondition(fake_tank_drive),
            ),
            ComposableNodeContainer(
                node_name="container",
                node_namespace="/boarai/hardware",
                package="rclcpp_components",
                node_executable="component_container_mt",
                composable_node_descriptions=[
                    GPS_PROVIDER,
                    TANK_DRIVE,
                    IMU,
                ],
                output="screen",
                emulate_tty=True,
                condition=UnlessCondition(fake_tank_drive),
            ),
            GAMEPAD,
        ]
    )
