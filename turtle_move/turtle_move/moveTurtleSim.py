from geometry_msgs.msg import Twist

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node

from turtlesim.msg import Pose


class T_move(Node):
    def __init__(self):
        super().__init__("turtleMove")  # type: ignore
        self.create_timer(0.01, self.pub_callback)
        self.create_timer(1 / 60, self.update_callback)
        self.pub = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.x = 0.0
        self.z = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.phase = 0
        self.prevTime = self.get_clock().now()

    def pub_callback(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.pub.publish(msg)

    def update_callback(self):
        if self.phase == 0:
            # 회전
            self.x = 0.0
            self.z = 2.0
            if (self.get_clock().now() - self.prevTime) > Duration(
                seconds=1, nanoseconds=250_000_000
            ):
                self.prevTime = self.get_clock().now()
                self.phase = 1
        elif self.phase == 1:
            # 진직
            self.x = 1.0
            self.z = 0.0
            if (self.get_clock().now() - self.prevTime) > Duration(seconds=2):
                self.prevTime = self.get_clock().now()
                self.phase = 0
        self.get_logger().info(f"phase: {self.phase}")

    def pose_callback(self, msg: Pose):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta


def main():
    rclpy.init()
    node = T_move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
