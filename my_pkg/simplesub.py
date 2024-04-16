import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleSub(Node):
    def __init__(self):
        super().__init__('simplesub') # type: ignore
        self.create_subscription(Twist, '/turtle1/cmd_vel', self.sub_callback, 10)

    def sub_callback(self, msg: Twist):
        print(f"linear.x: {msg.linear.x}")
        print(f"angular.z: {msg.angular.z}")

def main():
    rclpy.init()
    node = SimpleSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
