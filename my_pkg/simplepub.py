import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimplePub(Node):
    def __init__(self):
        super().__init__('simplepub') # type: ignore
        self.create_timer(1, self.print_hello)
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def print_hello(self):
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SimplePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
