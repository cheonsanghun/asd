import rclpy
from rclpy.node import Node

from custom_interface.msg import TwoNumbers
from std_msgs.msg import Int64
import math


class Calculator(Node):

    def __init__(self):
        super().__init__('calculator')
        self.subscription = self.create_subscription(
            TwoNumbers,
            'data',
            self.listener_callback,
            10
        )
        self.subscription
        self.publisher = self.create_publisher(Int64, 'result', 1)

    def listener_callback(self, msg: TwoNumbers):
        self.get_logger().info('I heard: "%s"' % msg)
        num1 = msg.a
        num2 = msg.b
        if msg.op == '+':
            result = num1 + num2
        elif msg.op == '-':
            result = num1 - num2
        elif msg.op == '*':
            result = num1 * num2
        elif msg.op == '/':
            result = num1 // num2 if num2 != 0 else math.inf
        #
        pub_msg = Int64()
        pub_msg.data = result
        self.publisher.publish(pub_msg)


def main(args=None):
    rclpy.init(args=args)
    calculator = Calculator()
    try:
        rclpy.spin(calculator)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()