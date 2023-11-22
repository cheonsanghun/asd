import rclpy
from rclpy.node import Node

from std_msgs.msg import Int64


class Display(Node):

    def __init__(self):
        super().__init__('display')
        self.subscription = self.create_subscription(
            Int64,
            'result',
            self.listener_callback,
            1)
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('result: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)
    display = Display()
    rclpy.spin(display)
    rclpy.shutdown()


if __name__ == '__main__':
    main()