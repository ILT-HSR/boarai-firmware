import launch.actions

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


POSITION_ESTIMATOR = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        get_package_share_directory('boarai_estimation') + '/launch/production.launch.py'
    )
)

HARDWARE_LAYER = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        get_package_share_directory('boarai_hardware') + '/launch/production.launch.py'
    )
)

INTERFACE_LAYER = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        get_package_share_directory('boarai_interface') + '/launch/production.launch.py'
    )
)

INTELLIGENCE_LAYER = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
        get_package_share_directory('boarai_intelligence') + '/launch/production.launch.py'
    )
)


def generate_launch_description():
    return LaunchDescription([
        POSITION_ESTIMATOR,
        HARDWARE_LAYER,
        INTERFACE_LAYER,
        INTELLIGENCE_LAYER,
    ])