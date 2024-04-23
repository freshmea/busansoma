import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from turtlesim.msg import Pose

MAX_VEL = 0.21
MAX_ANGLE = 2.84


class Tbot_move(Node):
    def __init__(self):
        super().__init__("turtleBotMove")  # type: ignore

        self.declare_parameter("max_vel", 0.21)
        self.declare_parameter("max_angle", 2.84)
        self.max_vel = self.get_parameter("max_vel").value
        self.max_angle = self.get_parameter("max_angle").value
        self.add_on_set_parameters_callback(self.param_update)

        laser_profile = qos_profile_sensor_data
        self.create_timer(0.01, self.pub_callback)
        self.create_timer(1 / 60, self.update_callback)
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_subscription(LaserScan, "scan", self.laser_callback, laser_profile)
        self.create_subscription(Odometry, "odom", self.odom_callback, 10)
        self.create_subscription(Imu, "imu", self.imu_callback, 10)
        self.create_subscription(BatteryState, "battery", self.battery_callback, 10)
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.laserScan = LaserScan()
        self.laserScan.ranges = [0.0]
        self.odom = Odometry()
        self.imu = Imu()
        self.battery = BatteryState()

    def laser_callback(self, msg: LaserScan):
        self.laserScan = msg

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def imu_callback(self, msg: Imu):
        self.imu = msg

    def battery_callback(self, msg: BatteryState):
        self.battery = msg

    def pub_callback(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.angular_velocity
        msg = self.restriction(msg)
        self.pub.publish(msg)

    def restriction(self, msg: Twist):
        msg.linear.x = min(MAX_VEL, msg.linear.x)
        msg.linear.x = max(-MAX_VEL, msg.linear.x)
        msg.angular.z = min(MAX_ANGLE, msg.angular.z)
        msg.angular.z = max(-MAX_ANGLE, msg.angular.z)
        return msg

    def update_callback(self):
        if self.laserScan.ranges[0] > 0.25:
            self.velocity = self.max_vel/2 #type: ignore
            self.angular_velocity = 0.0
        elif self.laserScan.ranges[0] < 0.2:
            self.velocity = -self.max_vel/2 #type: ignore
            self.angular_velocity = 0.0
        else:
            self.velocity = 0.0
            self.angular_velocity = 0.0

    def pose_callback(self, msg: Pose):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

    def param_update(self, params: list[Parameter]):
        for param in params:
            if param.name == "max_vel":
                self.max_vel = param.value
                self.get_logger().info(f"Updated param: {self.max_vel}")
            if param.name == "max_angle":
                self.max_angle = param.value
                self.get_logger().info(f"Updated param: {self.max_angle}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = Tbot_move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
