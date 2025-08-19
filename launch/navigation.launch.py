# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
import launch_ros.descriptions
from launch_ros.actions import Node


def generate_launch_description():

    # pkg_ros_gz_sim_demos = get_package_share_directory('ros_gz_sim_demos')
    pkg_robotino_navigation = get_package_share_directory('robotino_navigation')
    pkg_nav2 = get_package_share_directory('nav2_bringup')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time (true for simulation, false for real robot)'
    )
    
    nav_2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': [os.path.join(pkg_robotino_navigation, 'config', 'julio_params_test.yaml')],
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
             '-d', os.path.join(pkg_robotino_navigation, 'rviz', 'navigation.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([

        DeclareLaunchArgument('rviz', default_value='true',description='Open RViz.'),
        use_sim_time_arg,
        nav_2,
        rviz,
    ])