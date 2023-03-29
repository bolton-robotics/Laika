#####################################################################
# Filename: laika.rviz.launch.py                                    #
# Date: 12 March 2023                                               #
#                                                                   #
# Description: Launches RViz2, Robot State Publisher, Joint         #
#              State Publisher and Joint State Publisher GUI.       #
#              This will pick up a basic RViz2 configuration and    #
#              enable testing of URDF files with joint              #
#              manipulation.                                        #
#                                                                   #
#####################################################################


import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
  # Specify the file paths for Rviz2 and XACRO arguments
    
    rviz_file_name = 'laika.rviz'
    rviz = os.path.join(
        get_package_share_directory('laika_description'),
        rviz_file_name) 
    with open(rviz, 'r') as infp:
        robot_desc = infp.read()

    urdf_file_name = 'rviz.urdf.xacro'
    urdf = os.path.join(
        get_package_share_directory('laika_description'),
        urdf_file_name)

    doc = xacro.process_file(urdf)
    robot_desc = doc.toprettyxml(indent='  ')

  # Specify the actions
 
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_State_publisher',
      parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
      arguments=[urdf])
  
  # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
      package='joint_state_publisher',
      executable='joint_state_publisher',
      name='joint_state_publisher')
 
  # A GUI to manipulate the joint state values
    start_joint_state_publisher_gui_node = Node(
      package='joint_state_publisher_gui',
      executable='joint_state_publisher_gui',
      name='joint_state_publisher_gui')
 
  # Launch RViz
    start_rviz_cmd = Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      output='screen',
      arguments=['-d', rviz])
   
  # Create the launch description and populate
    ld = LaunchDescription()
 
   
  # Add any actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_node)
    ld.add_action(start_rviz_cmd)
    
    return ld
