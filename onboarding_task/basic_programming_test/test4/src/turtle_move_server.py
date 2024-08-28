import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from turtlesim.msg import Pose

from rclpy.executors import MultiThreadedExecutor

import math
import time

from test4.action import TurtleMove

class TurtleMoveActionServer(Node):
    def __init__(self):
        super().__init__('turtle_move_action_server')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self._action_server = ActionServer(
            self,
            TurtleMove,
            'start_move',
            self.execute_callback
        )
        self.current_position = Pose()
        self.subsciption = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )


    async def pose_callback(self, msg):
        # self.get_logger().info(f"Received pose update: x={msg.x}, y={msg.y}, theta={msg.theta}")
        self.current_position.x = round(msg.x, 4)
        self.current_position.y = round(msg.y, 4) 
        self.current_position.theta = msg.theta


    async def execute_callback(self, goal_handle):
        x = goal_handle.request.x
        y = goal_handle.request.y
        feedback_msg = TurtleMove.Feedback()
        feedback_msg.current_pose = self.current_position

        target_position = Point()
        target_position.x = float(x)
        target_position.y = float(y)

        while self.distance(self.current_position, target_position) > 0.1:
            twist_cmd = self.calculate_twist(target_position)
            self.publisher_.publish(twist_cmd)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = TurtleMove.Result()
        result.success = True
        return result

    def distance(self, p1, p2):
        return math.sqrt((p2.x - p1.x)**2 + (p2.y - p1.y)**2)

    def calculate_twist(self, target_position):
        twist = Twist()
        dx = target_position.x - self.current_position.x
        dy = target_position.y - self.current_position.y
        distance = self.distance(self.current_position, target_position)
        angle_to_target = math.atan2(dy, dx)
        error_angle = angle_to_target - self.current_position.theta
        twist.angular.z = 0.5 * error_angle
        if abs(error_angle) < 0.01:
            twist.linear.x = 1 * distance
        return twist

def main(args=None):
    rclpy.init(args=args)
    server = TurtleMoveActionServer()
    
    executor = MultiThreadedExecutor()
    rclpy.spin(server, executor=executor)


if __name__ == '__main__':
    main()