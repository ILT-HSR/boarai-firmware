from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    fake_drive = DeclareLaunchArgument(
        name="fake_tank_drive",
        default_value="False",
        description="Run with the faked boarai::hardware::tank_drive node",
    )

    return LaunchDescription(
        [
            # Arguments
            fake_drive,
            # Sub-launch files
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("boarai_control")
                    + "/launch/production.launch.py"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("boarai_estimation")
                    + "/launch/production.launch.py"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("boarai_hardware")
                    + "/launch/production.launch.py"
                ),
                launch_arguments={
                    "fake_tank_drive": LaunchConfiguration("fake_tank_drive"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("boarai_interface")
                    + "/launch/production.launch.py"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("boarai_intelligence")
                    + "/launch/production.launch.py"
                )
            ),
        ]
    )
