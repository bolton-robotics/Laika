# Copyright 2024 Andrew Poole
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


# Package is for use on the Laika RPi4b.  It is not to be run on
# the workstation.  It loads the following software:
# - RPi Camera
# - RP Lidar
# - ADXL IMU (TBA)
# - Pixy2Cam (TBA)
# - ROS2 Control Plugins
#
# Ensure that the Arduino code has been loaded and installation
# pre-requisites have been met prior to running


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Get share directory paths.
pkg_laika_bringup = get_package_share_directory('laika_bringup')
pkg_laika_control = get_package_share_directory('laika_control')
pkg_laika_description = get_package_share_directory('laika_description')

def generate_launch_description():
    # Declares launch arguments
    camera_arg = DeclareLaunchArgument(
            'include_camera',
            default_value='True',
            description='Indicates whether to include camera launch.')
    camera =  LaunchConfiguration('include_camera')
    rplidar_arg = DeclareLaunchArgument(
            'include_rplidar',
            default_value='True',
            description='Indicates whether to include rplidar launch.')
    rplidar =  LaunchConfiguration('include_rplidar')
    imu_arg = DeclareLaunchArgument(
            'include_imu',
            default_value='False',
            description='Indicates whether to include IMU launch.')
    imu =  LaunchConfiguration('include_imu')
    pixycam_arg = DeclareLaunchArgument(
            'include_pixycam',
            default_value='False',
            description='Indicates whether to include Pixy2Cam launch.')
    pixycam =  LaunchConfiguration('include_pixycam')    

    # Includes laika_description launch file
    include_laika_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_description, 'launch', 'laika_description.launch.py'),
        ),
        launch_arguments={
            'rsp': 'True',
        }.items()
    )

    # Include laika_control launch file
    include_laika_control =  IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_control, 'launch', 'laika_control.launch.py'),
        ),
        launch_arguments={
        }.items()
    )

    # Include rplidar launch file
    include_rplidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_bringup, 'launch', 'rplidar.launch.py'),
        ),
        launch_arguments={
            "serial_port": '/dev/ttyUSB_LIDAR',
        }.items(),
                condition=IfCondition(rplidar)
    )
    
    # Include camera launch file
    include_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_bringup, 'launch', 'camera.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(camera)
    )

    
    # Include IMU launch file
    include_imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_bringup, 'launch', 'imu.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(imu)
    )

    
    # Include Pixy2Cam launch file
    include_pixycam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_laika_bringup, 'launch', 'pixycam.launch.py'),
        ),
        launch_arguments={
        }.items(),
                condition=IfCondition(pixycam)
    )


    # Waits for laika_description to set up robot_state_publisher.
    laika_control_timer = TimerAction(period=5.0, actions=[include_laika_control])
    # Defer sensors launch to avoid overhead while robot_state_publisher is setting up.
    rplidar_timer = TimerAction(period=3.0, actions=[include_rplidar])
    camera_timer = TimerAction(period=3.0, actions=[include_camera])
    imu_timer = TimerAction(period=3.0, actions=[include_imu])
    pixycam_timer = TimerAction(period=3.0, actions=[include_pixycam])

    return LaunchDescription([
        include_laika_description,
        laika_control_timer,
        camera_arg,
        camera_timer,
        rplidar_arg,
        rplidar_timer,
        imu_arg,
        imu_timer,
        pixycam_arg,
        pixycam_timer,
    ])
