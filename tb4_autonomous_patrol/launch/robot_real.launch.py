from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory(
            'tb4_autonomous_patrol'))/'launch/tb4_system.launch.py')),
        launch_arguments={'mode': 'real'}.items())])
