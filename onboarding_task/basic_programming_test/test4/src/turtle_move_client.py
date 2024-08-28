import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from rclpy.executors import MultiThreadedExecutor

import math
import time

from test4.action import TurtleMove


class TurtleMoveActionClient(Node):
    def __init__(self):
        super().__init__('turtle_move_action_client')
        self._action_client = ActionClient(
            self,
            TurtleMove,
            'start_move'
        )

    def send_goal(self, x, y):
        goal_msg = TurtleMove.Goal()
        print(x, y)

        goal_msg.x = x
        goal_msg.y = y
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Moved {result}')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback! x: {feedback.current_pose.x}, y: {feedback.current_pose.y}, theta: {feedback.current_pose.theta}')

def main(args=None):
    rclpy.init(args=args)
    client = TurtleMoveActionClient()
    x, y = 10, 10

    future = client.send_goal(x, y)
    rclpy.spin(client)


if __name__ == '__main__':
    main()