import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class SimpleTimeSub(Node):
    def __init__(self):
        super().__init__('simpleTimeSub') # type: ignore
        self.pub = self.create_subscription(Header, 'timeTopic', self.sub_callback, 10)

    def sub_callback(self, msg: Header):
        msg = Header()
        print(f"frame_id: {msg.frame_id}")
        print(f"stamp: {msg.stamp.sec}.{msg.stamp.nanosec}")

def main():
    rclpy.init()
    node = SimpleTimeSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
