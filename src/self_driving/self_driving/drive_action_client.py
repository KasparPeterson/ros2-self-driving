import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from self_driving_action_interfaces.action import Drive


class DriveActionClient(Node):

    def __init__(self):
        super().__init__('drive_action_client')
        self._action_client = ActionClient(self, Drive, 'drive')

    def send_goal(self, x: float, y: float):
        goal_msg = Drive.Goal()
        goal_msg.target_x = x
        goal_msg.target_y = y

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.get_logger().info('Goal response callback')
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Goal result callback')
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.total_distance))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_x))
        self.get_logger().info(f'Received feedback, x: {feedback.current_x}, y: {feedback.current_x}')


def main(args=None):
    rclpy.init(args=args)
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    action_client = DriveActionClient()
    action_client.send_goal(x, y)
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
