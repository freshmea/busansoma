import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# import Empty service
from std_srvs.srv import SetBool, Trigger


class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__("simpleservice")  # type: ignore
        self.create_service(SetBool, "setBool", self.setBool_callback)

    def setBool_callback(self, request: SetBool.Request, response : SetBool.Response):
        self.get_logger().info(f"request data: {request.data}")
        if request.data:
            response.success = True
            response.message = "True request"
        else:
            response.success = False
            response.message = "False request"
        return response


def main():
    rclpy.init()
    node = SimpleServiceServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
