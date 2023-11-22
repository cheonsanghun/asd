import rclpy
from rclpy.node import Node

from custom_interface.msg import TwoNumbers
import random


class Sender(Node):
    def __init__(self):
        super().__init__('sender')
        self.publisher_ = self.create_publisher(TwoNumbers, 'data', 1)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = TwoNumbers()
        msg.a = random.randrange(100)
        msg.b = random.randrange(100)
        msg.op = random.choice(['+', '-', '*', '/'])
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)
    sender = Sender()
    try:
        rclpy.spin(sender)
    except KeyboardInterrupt:
        sender.get_logger().info('User-terminated by pressing ctrl-c...')
    rclpy.shutdown()


if __name__ == '__main__':
    main()