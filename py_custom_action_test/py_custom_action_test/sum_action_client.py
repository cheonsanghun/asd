import rclpy
from rclpy.action import ActionClient
from rclpy.action.client import Future
from rclpy.node import Node

from custom_interface.action import Sum
import sys


class SumActionClient(Node):
    def __init__(self):
        super().__init__('sum_action_client')
        self.action_client = ActionClient(self, Sum, 'sum')
        self.send_goal_future = None

    def send_goal(self, n):
        goal_msg = Sum.Goal()
        goal_msg.n = n
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self.send_goal_future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future: Future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_cb)

    def get_result_cb(self, future: Future):
        result = future.result().result
        sum = result.sum
        self.get_logger().info('Sum = %d' % sum)
        rclpy.shutdown()

    def feedback_cb(self, msg: Sum.Impl.FeedbackMessage):
        feedback = msg.feedback
        self.get_logger().info('received feedback: %.3f' % feedback.elapsed_time)


def main(args=None):
    rclpy.init(args=args)
    n = int(sys.argv[1])
    client = SumActionClient()
    client.send_goal(n)
    rclpy.spin(client)


if __name__ == '__main__':
    main()
