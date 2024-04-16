import rclpy
from rclpy.node import Node
from std_msgs.msg import Header

class SimpleTimePub(Node):
    def __init__(self):
        super().__init__('simpleTimePub') # type: ignore
        self.create_timer(1, self.timer_callback)
        self.pub = self.create_publisher(Header, 'timeTopic', 10)
        self.count = 0

    def timer_callback(self):
        msg = Header()
        msg.frame_id = 'time'+str(self.count)
        msg.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)
        self.count += 1

def main():
    rclpy.init()
    node = SimpleTimePub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
