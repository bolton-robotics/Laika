# Programme to move robot based on cmd_vel message from Twist in Python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import RPi.GPIO as GPIO            # import RPi.GPIO module  
from time import sleep             # lets us have a delay 
GPIO.cleanup()                     # resets all GPIO ports used by this program 
GPIO.setmode(GPIO.BCM)             # choose BCM or BOARD  
GPIO.setup(10, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(9, GPIO.OUT)            # set GPIO24 as an output   
GPIO.setup(11, GPIO.OUT)           # set GPIO24 as an output   
GPIO.setup(17, GPIO.OUT)           # set GPIO24 as an output
GPIO.setup(27, GPIO.OUT)           # set GPIO24 as an output   
GPIO.setup(22, GPIO.OUT)           # set GPIO24 as an output   

class LaikaMotorControl(Node):

    def __init__(self):
        super().__init__('laika_motor_control')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def forwards(self):
        GPIO.output(27, 1)         # set AI2 to 1/GPIO.HIGH/True
        GPIO.output(22, 0)         # set AI1 to 0/GPIO.LOW/False
        GPIO.output(17, 1)         # set PWMA to 1/GPIO.HIGH/True
        GPIO.output(10, 0)         # set BI1 to 0/GPIO.LOW/False
        GPIO.output(9, 1)          # set BI2 to 1/GPIO.HIGH/True
        GPIO.output(11, 1)         # set PWMB to 1/GPIO.HIGH/True
        sleep(0.2)
        GPIO.output(17, 0)         # set PWMA to 0/GPIO.LOW/False
        GPIO.output(11, 0)         # set PWMB to 0/GPIO.LOW/False

    def backwards(self):
        GPIO.output(22, 1)         # set AI1 to 1/GPIO.HIGH/True
        GPIO.output(27, 0)         # set AI2 to 0/GPIO.LOW/False
        GPIO.output(17, 1)         # set PWMA to 1/GPIO.HIGH/True
        GPIO.output(9, 0)         # set BI2 to 0/GPIO.LOW/False
        GPIO.output(10, 1)          # set BI1 to 1/GPIO.HIGH/True
        GPIO.output(11, 1)         # set PWMB to 1/GPIO.HIGH/True
        sleep(0.2)
        GPIO.output(17, 0)         # set PWMA to 0/GPIO.LOW/False
        GPIO.output(11, 0)         # set PWMB to 0/GPIO.LOW/False

    def right(self):
        GPIO.output(27, 1)         # set AI2 to 1/GPIO.HIGH/True
        GPIO.output(22, 0)         # set AI1 to 0/GPIO.LOW/False
        GPIO.output(17, 1)         # set PWMA to 1/GPIO.HIGH/True
        sleep(0.2)
        GPIO.output(17, 0)         # set PWMA to 0/GPIO.LOW/False
        GPIO.output(11, 0)         # set PWMB to 0/GPIO.LOW/False

    def left(self):
        GPIO.output(10, 0)         # set BI1 to 0/GPIO.LOW/False
        GPIO.output(9, 1)          # set BI2 to 1/GPIO.HIGH/True
        GPIO.output(11, 1)         # set PWMB to 1/GPIO.HIGH/True
        sleep(0.2)
        GPIO.output(17, 0)         # set PWMA to 0/GPIO.LOW/False
        GPIO.output(11, 0)         # set PWMB to 0/GPIO.LOW/False

    def stop(self):
        GPIO.output(17, 0)         # set PWMA to 0/GPIO.LOW/False
        GPIO.output(11, 0)         # set PWMB to 0/GPIO.LOW/False

    def listener_callback(self, msg):
        #self.get_logger().info("Received a /cmd_vel message!")
        #self.get_logger().info("Linear Components: [%f, %f, %f]"%(msg.linear.x,
 msg.linear.y, msg.linear.z))
        #self.get_logger().info("Angular Components: [%f, %f, %f]"%(msg.angular.
x, msg.angular.y, msg.angular.z))

        if msg.linear.x > 0:
            self.forwards()
            self.get_logger().info("Forwards")
        elif msg.linear.x < 0:
            self.backwards()
            self.get_logger().info("Backwards")

        if msg.angular.z > 0:
            self.right()
            self.get_logger().info("Right")
        elif msg.angular.z < 0:
            self.left()
            self.get_logger().info("Left")

        self.stop()
        self.get_logger().info("Stop")


        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities 
into motor commands

def main(args=None):
    rclpy.init(args=args)

    laika_motor_control = LaikaMotorControl()

    rclpy.spin(laika_motor_control)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laika_motor_control.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()                     # resets all GPIO ports used by this prog
ram


if __name__ == '__main__':
    main()

