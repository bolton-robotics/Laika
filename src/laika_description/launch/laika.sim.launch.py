#####################################################################
# Filename: laika.gazebo.launch.py                                  #
# Date: 12 March 2023                                               #
#                                                                   #
# Description: Launches Gazebo, RViz2, Teleop Twist Keyboard as     #
#              well as the Gazebo 'basic' world and 'odom' RViz2    #
#              configuration.  Robot State Publisher is launched.   #
#              Gazebo provides skid steer (diff drive) config, as   #
#              well as Joint States, IMU, Lidar and Camera feeds.   #
#                                                                   #
#####################################################################

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
  # Specify the file paths for Rviz2 and XACRO arguments
    rviz_file_name = 'laika.gazebo.rviz'
    rviz = os.path.join(
        get_package_share_directory('laika_description'),
        rviz_file_name)
    with open(rviz, 'r') as infp:
        robot_desc = infp.read()

    urdf_file_name = 'sim.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('laika_description'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.00'
    
    # Constants for paths to different files and folders
    package_name = 'laika_description'
    robot_name_in_model = 'laika'
    robot_world = 'laika.basic.world'
    robot_world_name = os.path.join(
        get_package_share_directory('laika_description'),
        robot_world)
    rviz_config_file_path = os.path.join(get_package_share_directory('laika_description'))
    urdf_file_path = os.path.join(get_package_share_directory('laika_description'))
    world_file_path = os.path.join(get_package_share_directory('laika_description'))
  

    # Set the path to different files and folders.  
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    pkg_share = FindPackageShare(package=package_name).find(package_name)
    default_urdf_model_path = os.path.join(get_package_share_directory('laika_description'))
    default_rviz_config_path = os.path.join(get_package_share_directory('laika_description'))
    world_path = os.path.join(get_package_share_directory('laika_description'))
    gazebo_models_path = os.path.join(get_package_share_directory('laika_description'))
    os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
     
  # Specify the actions

  # Start Gazebo server
  #  start_gazebo_server_cmd = IncludeLaunchDescription(
  #  PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py'))
  #  ,launch_arguments={'world': robot_world_name}.items())
 
  # Start Gazebo client    
  #  start_gazebo_client_cmd = IncludeLaunchDescription(
  #  PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))

  # Start Gazebo

    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')

    # Include the Gazebo launch file, provided by the gazebo_ros package
    start_gazebo_cmd = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
             )
 
  # Launch the robot
    start_spawn_entity_cmd = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
                '-topic', 'robot_description',
                    '-x', spawn_x_val,
                    '-y', spawn_y_val,
                    '-z', spawn_z_val,
                    '-Y', spawn_yaw_val],
                    output='screen')
 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_State_publisher',
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
      arguments=[urdf])
  
  # Joint State Publisher is not launched as there is a Gazebo Plugin that handles this.  Launching
  # Joint State Publisher here will cause an error.

  # Diff Drive and Joint State Broadcaster

    start_diff_drive_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont'],
    )

    start_joint_broad_cmd = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad'],
    )

  # Joystick and Twist_Mux
    joy_params = os.path.join(get_package_share_directory('laika_description'),'joystick.yaml')

    start_joystick_cmd = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
         )

    twist_mux_params = os.path.join(get_package_share_directory(package_name),'twist_mux.yaml')

    start_twist_mux_cmd = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )



    start_teleop_cmd = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params, {'use_sim_time': use_sim_time}],
            remappings=[('/cmd_vel','/cmd_vel_joy')]
         )


 
  # Launch RViz
    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz])

  # Run Teleoperations Keyboard Node.
  #  start_teleop_cmd = Node(
  #  package='teleop_twist_keyboard',
  #  executable='teleop_twist_keyboard',
  #  name='teleop_twist_keyboard')
   
  # Create the launch description and populate
    ld = LaunchDescription()
   
  # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joystick_cmd)
    ld.add_action(start_teleop_cmd)
    ld.add_action(start_twist_mux_cmd)
    ld.add_action(start_gazebo_cmd)
  # ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_spawn_entity_cmd)
    ld.add_action(start_diff_drive_cmd)
    ld.add_action(start_joint_broad_cmd)
  #  ld.add_action(start_rviz_cmd)
  # ld.add_action(start_teleop_cmd)
    
    return ld
