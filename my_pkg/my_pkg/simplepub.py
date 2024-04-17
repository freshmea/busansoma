import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePub(Node):
    def __init__(self):
        super().__init__('simplepub') # type: ignore
        self.create_timer(1, self.print_hello)
        self.pub = self.create_publisher(String, 'helloWorld', 10)
        self.count = 0

    def print_hello(self):
        msg = String()
        msg.data = 'hello, world'+ str(self.count)
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
