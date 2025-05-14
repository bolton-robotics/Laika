from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld=LaunchDescription()

    camera_node=Node(
        package="v4l2_camera",
        executable="v4l2_camera_node",
        parameter=[
          {"image_size":"[640,480]",
          "camera_frame_id":"camera_optical_link"}
        ]

    )

    lidar_node=Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        parameter=[
            "serial_port":"/dev/ttyUSB0",
            "frame_id":"laser_frame",
            "angle_compensate":True,
            "serial_baudrate":115200}
        ]

    ld.add_action(camera_node)
    ld.add_action(lidar_node)

    return ld

    )
    
    