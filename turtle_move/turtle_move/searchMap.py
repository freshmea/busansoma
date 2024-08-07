import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from std_srvs.srv import SetBool


class FollowWall(Node):
    def __init__(self):
        super().__init__("followWall")  # type: ignore

        self.declare_parameter("max_vel", 0.11)
        self.declare_parameter("max_angle", 1.84)
        self.declare_parameter("max_slice", 8)
        self.max_vel = self.get_parameter("max_vel").value
        self.max_angle = self.get_parameter("max_angle").value
        self.max_slice : int = self.get_parameter("max_slice").value # type: ignore
        self.add_on_set_parameters_callback(self.param_update)

        self.create_service(SetBool, "stopturtle", self.stop_callback)
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
        self.scan_avg = list([3.5 for i in range(self.max_slice)]) # type: ignore
        self.odom = Odometry()
        self.imu = Imu()
        self.battery = BatteryState()
        self.wall_detect = False
        self.stopGo = True
        self.prevTime = self.get_clock().now()
        self.phase = 0
        # 랜덤 회전 시간 1~3 초 float
        self.rotation_random_time = np.random.uniform(1, 5)

    def laser_callback(self, msg: LaserScan):
        self.laserScan = msg
        for i, data in enumerate(self.laserScan.ranges):
            if data == 0.0:
                self.laserScan.ranges[i] = 3.5
        for i in range(self.max_slice):
            self.scan_avg[i] = float(np.average(self.laserScan.ranges[int(360/self.max_slice*i):int(360/self.max_slice*(i+1)-1)]))

    def odom_callback(self, msg: Odometry):
        self.odom = msg

    def imu_callback(self, msg: Imu):
        self.imu = msg

    def battery_callback(self, msg: BatteryState):
        self.battery = msg

    def stop_callback(self, request, response):
        self.stopGo = request.data
        return response

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
        if self.stopGo:
            if self.phase == 0:
                self.get_logger().info("Phase 0")
                # 벽을 찾은 후에 벽을 따라감 찾기전에는 직진
                if self.wall_detect:
                    # 정면에 장애물이 있을 때
                    if (self.scan_avg[0] < 0.4) or (self.scan_avg[7] < 0.4):
                        self.velocity = 0.0
                        self.angular_velocity = -self.max_angle / 5 # type: ignore
                    else:
                        # 벽을 따라 일정 거리 유지하면서 이동
                        if self.scan_avg[1] > 0.4:
                            self.velocity = self.max_vel / 2 # type: ignore
                            self.angular_velocity = self.max_angle / 5 # type: ignore
                        elif self.scan_avg[1] < 0.3:
                            self.velocity = self.max_vel / 2 # type: ignore
                            self.angular_velocity = -self.max_angle / 5 # type: ignore
                        else:
                            self.velocity = self.max_vel
                            self.angular_velocity = 0.0
                else:
                    self.velocity = self.max_vel
                    self.angular_velocity = 0.0
                    # 정면에 벽을 찾았을 때
                    if (self.scan_avg[0] < 0.4) or (self.scan_avg[7] < 0.4):
                        self.wall_detect = True
                        self.get_logger().info("Wall Detected")
                # 시간 체크
                if self.get_clock().now() - self.prevTime > Duration(seconds=30):
                    self.phase = 1
                    self.prevTime = self.get_clock().now()
                    self.rotation_random_time = np.random.uniform(1, 5)
            else:
                self.get_logger().info("Phase 1")
                # 랜덤 시간 만큼 으로 회전
                if (self.get_clock().now() - self.prevTime) < Duration(seconds=int(self.rotation_random_time), nanoseconds=int((self.rotation_random_time - int(self.rotation_random_time)) * 1e9)):
                    self.velocity = 0.0
                    self.angular_velocity = self.max_angle / 5 # type: ignore
                    self.get_logger().info(f"Rotation Time: {self.rotation_random_time}")
                else:
                    # 직진
                    self.velocity = self.max_vel
                    self.angular_velocity = 0.0
                    self.get_logger().info("Go Straight")
                    # 앞에 장애물이 있을 때 회전
                    if (self.scan_avg[0] < 0.5) or (self.scan_avg[7] < 0.5):
                        self.velocity = 0.0
                        self.angular_velocity = self.max_angle / 5 # type: ignore
                        self.get_logger().info("Obstacle Detected")
                # 시간 체크
                if self.get_clock().now() - self.prevTime > Duration(seconds=30):
                    self.phase = 0
                    self.prevTime = self.get_clock().now()
        else:
            self.velocity = 0.0
            self.angular_velocity = 0.0


    def param_update(self, params: list[Parameter]):
        for param in params:
            if param.name == "max_vel":
                self.max_vel = param.value
                self.get_logger().info(f"Updated param: {self.max_vel}")
            if param.name == "max_angle":
                self.max_angle = param.value
                self.get_logger().info(f"Updated param: {self.max_angle}")
            if param.name == "max_slice":
                self.max_slice = param.value # type: ignore
                self.get_logger().info(f"Updated param: {self.max_slice}")
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
