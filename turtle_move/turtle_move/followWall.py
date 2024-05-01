import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from turtlesim.msg import Pose


class FollowWall(Node):
    def __init__(self):
        super().__init__("followWall")  # type: ignore

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
        self.create_subscription(BatteryState, "battery_state", self.battery_callback, 10)
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.left = 0.0
        self.lefttop = 0.0
        self.leftbottom = 0.0
        self.laserScan = LaserScan()
        self.laserScan.ranges = [0.0]
        self.odom = Odometry()
        self.imu = Imu()
        self.battery = BatteryState()

    def laser_callback(self, msg: LaserScan):
        self.laserScan = msg
        self.left = self.laserScan.ranges[90]
        self.lefttop = self.laserScan.ranges[45]
        self.leftbottom = self.laserScan.ranges[135]

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
        msg.linear.x = min(self.max_vel, msg.linear.x) #type: ignore
        msg.linear.x = max(-self.max_vel, msg.linear.x) #type: ignore
        msg.angular.z = min(self.max_angle, msg.angular.z) #type: ignore
        msg.angular.z = max(-self.max_angle, msg.angular.z) #type: ignore
        return msg

    def update_callback(self):
        # if self.laserScan.ranges[0] > 0.25:
        #     self.velocity = self.max_vel/2 #type: ignore
        #     self.angular_velocity = 0.0
        # elif self.laserScan.ranges[0] < 0.2:
        #     self.velocity = -self.max_vel/2 #type: ignore
        #     self.angular_velocity = 0.0
        # else:
        #     self.velocity = 0.0
        #     self.angular_velocity = 0.0
        self.velocity = self.max_vel
        self.ratio = 0.0
        try:
            self.ratio = self.lefttop / self.left
        except ZeroDivisionError:
            self.ratio = 0.0
        # if  self.ratio  == 1.414:
        #     self.angular_velocity = 0.0
        # elif self.ratio < 1.414:
        #     self.angular_velocity = -self.max_angle / 5  #type: ignore
        # elif self.ratio > 1.414:
        #     self.angular_velocity = self.max_angle / 5 #type: ignore

        if self.left < 0.4:
            self.angular_velocity = -self.max_angle / 5 #type: ignore
        elif self.left > 0.6:
            if self.ratio < 1.414:
                self.angular_velocity = -self.max_angle / 5  #type: ignore
            elif self.ratio > 1.414:
                self.angular_velocity = self.max_angle / 5 #type: ignore
        else:
            self.angular_velocity = 0.0


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
    node = FollowWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()


if __name__ == "__main__":
    main()
