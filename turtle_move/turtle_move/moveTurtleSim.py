import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class T_move(Node):
    def __init__(self):
        super().__init__('turtleMove') # type: ignore
        self.create_timer(0.01, self.pub_callback)
        self.create_timer(1/60, self.update_callback)
        self.pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        self.x = 0.0
        self.z = 0.0
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

    def pub_callback(self):
        msg = Twist()
        msg.linear.x = self.x
        msg.angular.z = self.z
        self.pub.publish(msg)

    def update_callback(self):
        self.x += 0.01
        if self.x > 2:
            self.x = 0.0
        self.z = 1.0

    def pose_callback(self, msg: Pose):
        self.get_logger().info(f"{msg.x}, {msg.y}, {msg.theta}")
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

if __name__ == '__main__':
    main()
