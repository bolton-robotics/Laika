#####################################################################
# Filename: laika.sim.launch.py                                  #
# Date: 1 May 2023                                               #
#                                                                   #
# Description: Launches Gazebo, RViz2, Teleop Twist Keyboard as     #
#              well as the Gazebo 'basic' world and 'odom' RViz2    #
#              configuration.  Robot State Publisher is launched.   #
#              Gazebo provides skid steer (diff drive) config, as   #
#              well as Joint States, IMU, Lidar and Camera feeds.   #
#                                                                   #
#####################################################################

import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xacro


def generate_launch_description():
    
    robot_world = 'laika.basic.world'
    robot_world_name = os.path.join(
        get_package_share_directory('laika_description'),
        robot_world)
    
# Specify the file paths for Rviz2 and XACRO arguments
    rviz_file_name = 'laika.gazebo.rviz'
    rviz = os.path.join(
        get_package_share_directory('laika_description'),
        rviz_file_name)
    with open(rviz, 'r') as infp:
        robot_desc = infp.read()
    
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                    launch_arguments={'world': robot_world_name}.items()
             )

    laika_description_path = os.path.join(
        get_package_share_directory('laika_description'))
    
    xacro_file = os.path.join(laika_description_path,'sim.urdf.xacro')

    doc = xacro.process_file(xacro_file)
    params = doc.toprettyxml(indent='  ')

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': params}],
        arguments=[xacro_file]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'diffbot'],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_base_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diff_drive_controller'],
        output='screen'
    )

  # Joystick and Twist_Mux
    joy_params = os.path.join(get_package_share_directory('laika_description'),'joystick.yaml')

    start_joystick_cmd = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': True}],
         )

    twist_mux_params = os.path.join(get_package_share_directory('laika_description'),'twist_mux.yaml')

    start_twist_mux_cmd = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_drive_controller/cmd_vel_unstamped')]
        )

    start_teleop_cmd = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )
 
   # Launch RViz

    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz])

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
               target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_diff_drive_base_controller],
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        start_joystick_cmd,
        start_twist_mux_cmd,
        start_teleop_cmd,
        start_rviz_cmd
    ])