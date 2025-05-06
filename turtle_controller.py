#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.target_x = 2.0
        self.target_y = 8.0
        self.pose_: Pose = None
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_pose, 10)
        self.control_loop_timer_ = self.create_timer(
            0.01, self.control_loop)

    def callback_pose(self, pose: Pose):
        self.pose_ = pose
        # Print the pose values
        self.get_logger().info(
            f'Pose -> x: {pose.x:.2f}, y: {pose.y:.2f}, theta: {pose.theta:.2f}, '
            f'linear_velocity: {pose.linear_velocity:.2f}, angular_velocity: {pose.angular_velocity:.2f}')

    def control_loop(self):
        if self.pose_ is None:
            return

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        goal_theta = math.atan2(dist_y, dist_x)
        heading_error = goal_theta - self.pose_.theta

        # Normalize angle between -pi and pi
        if heading_error > math.pi:
            heading_error -= 2 * math.pi
        elif heading_error < -math.pi:
            heading_error += 2 * math.pi

        cmd = Twist()

        # Phase 1: Rotate until facing the target
        if abs(heading_error) > 0.05:
            cmd.angular.z = 2.0 * heading_error  # Proportional turning
            cmd.linear.x = 0.0

        # Phase 2: Move forward once facing target
        elif distance > 0.1:
            cmd.linear.x = 1.0  # Constant speed
            cmd.angular.z = 0.0

        # Phase 3: Stop when close
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_vel_publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
