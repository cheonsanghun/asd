import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from custom_interface.action import Sum
from rclpy.action. server import ServerGoalHandle
import time


class SumActionServer (Node):
    def __init__(self):
        super().__init__('sum_action_server')
        self.action_server = ActionServer(self, Sum, 'sum', self.execute_callback)

    def execute_callback(self, goal_handle: ServerGoalHandle):
        n = goal_handle.request.n

        sum = 0
        start_time = self.get_clock().now()
        for i in range(1, n+1):
            time.sleep(1.0)
            sum += i
            current_time = self.get_clock().now()
            feedback_msg = Sum.Feedback()
            feedback_msg.elapsed_time = (current_time - start_time).nanoseconds / 1e9
            self.get_logger().info('elapsed time = %.3f' % feedback_msg.elapsed_time)
            goal_handle.publish_feedback(feedback_msg)

        goal_handle.succeed()
        result = Sum.Result()
        result.sum = sum
        return result


def main(args=None):
    rclpy.init(args=args)
    server = SumActionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()




