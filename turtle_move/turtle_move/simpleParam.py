import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter


class ParamNode(Node):
    def __init__(self):
        super().__init__('paramNode') # type: ignore
        self.declare_parameter('myparam', "내가 만든 파라미터")
        self.myparam = ""
        self.create_timer(1, self.print_param)
        self.add_on_set_parameters_callback(self.param_update)

    def print_param(self):
        self.get_logger().info(f"{self.myparam}")

    def param_update(self, params: list[Parameter]):
        for param in params:
            if param.name == "myparam":
                self.myparam = param.value
                self.get_logger().info(f"Updated param: {self.myparam}")
        return SetParametersResult(successful=True)



def main():
    rclpy.init()
    node = ParamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == '__main__':
    main()
