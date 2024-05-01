import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import Empty service
from std_srvs.srv import SetBool, Trigger


class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simpleServiceClient")  # type: ignore
        self.client = self.create_client(SetBool, "setBool")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.create_timer(1, self.send_request)
        self.data = SetBool.Request()

    def send_request(self):
        self.get_logger().info("send request")
        if self.data.data:
            self.data.data = False
        else:
            self.data.data = True
        self.future = self.client.call_async(self.data)
        self.future.add_done_callback(self.future_callback)

    def future_callback(self, future):
        response = future.result()
        self.get_logger().info(f"response : {response.message}")
        self.get_logger().info(f"success : {response.success}")


def main():
    rclpy.init()
    node = SimpleServiceClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
