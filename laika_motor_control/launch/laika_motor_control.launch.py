from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.node(
        package='laika_motor_control', node_executable='laika_motor_control', output='screen')
    ])