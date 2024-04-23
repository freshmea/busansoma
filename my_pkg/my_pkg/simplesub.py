import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Header, String


class SimpleSub(Node):
    def __init__(self):
        super().__init__("simplesub")  # type: ignore
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )
        self.create_subscription(String, "helloWorld", self.sub_callback, 10)
        self.create_subscription(Header, "timeTopic", self.time_callback, qos_profile)

    def sub_callback(self, msg: String):
        self.get_logger().info(f"Reciving msg from simiplepub : {msg.data}")

    def time_callback(self, msg: Header):
        self.get_logger().info(f"frame_id: {msg.frame_id}")
        self.get_logger().info(f"stamp: {msg.stamp.sec}.{msg.stamp.nanosec}")


def main():
    rclpy.init()
    node = SimpleSub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
