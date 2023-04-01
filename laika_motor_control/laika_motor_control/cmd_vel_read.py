# Programme to read cmd_vel message from Twist in Python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class LaikaMotorControl(Node):

    def __init__(self):
        super().__init__('laika_motor_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info("Received a /cmd_vel message!")
        self.get_logger().info("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        self.get_logger().info("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands

def main(args=None):
    rclpy.init(args=args)

    laika_motor_control = LaikaMotorControl()

    rclpy.spin(laika_motor_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laika_motor_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()