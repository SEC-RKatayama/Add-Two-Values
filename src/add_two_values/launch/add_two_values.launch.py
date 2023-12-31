import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
   library_path = get_package_share_directory('add_two_values') + '/release'

   return LaunchDescription([
      SetEnvironmentVariable(name='LD_LIBRARY_PATH', value=[EnvironmentVariable('LD_LIBRARY_PATH'), ':' + library_path]),
      Node(
         package='add_two_values',
         executable='add_two_values_exe',
         output="screen",
      )
   ])