import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from sensor_msgs.msg import BatteryState

MAX_VEL = 0.21
MAX_ANGLE = 2.84




class Tbot_move(Node):
    def __init__(self):
        super().__init__('turtleBotMove') # type: ignore
        laser_profile = sensor_profile
        self.create_timer(0.01, self.pub_callback)
        self.create_timer(1/60, self.update_callback)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.laser_callback, laser_profile)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Imu, 'imu', self.imu_callback, 10)
        self.create_subscription(BatteryState, 'battery', self.battery_callback, 10)
        self.velocity = 0.0
        self.angular_velocity = 0.0
        self.laserScan = LaserScan()
        self.odom = Odometry()
        self.imu = Imu()
        self.battery = BatteryState()
        
    def laser_callback(self, msg: LaserScan):
        self.laserScan = msg
        self.get_logger().info(f"laser: {msg.ranges[0]}")
        
    def odom_callback(self, msg: Odometry):
        self.odom = msg
        self.get_logger().info(f"odom: {msg.pose.pose.position.x}")

    def imu_callback(self, msg: Imu):
        self.imu = msg
        self.get_logger().info(f"imu: {msg.orientation.x}")
        
    def battery_callback(self, msg: BatteryState):
        self.battery = msg
        self.get_logger().info(f"battery: {msg.percentage}")
        
    def pub_callback(self):
        msg = Twist()
        msg.linear.x = self.velocity
        msg.angular.z = self.angular_velocity
        msg = self.restriction(msg)
        self.pub.publish(msg)
        
    def restriction(self, msg: Twist):
        # check +MAX_VEL 
        msg.linear.x = min(MAX_VEL, msg.linear.x)
        # check -MAX_VEL
        msg.linear.x = max(-MAX_VEL, msg.linear.x)
        # check +MAX_ANGLE
        msg.angular.z = min(MAX_ANGLE, msg.angular.z)
        # check -MAX_ANGLE
        msg.angular.z = max(-MAX_ANGLE, msg.angular.z)
        return msg

    def update_callback(self):
        if self.phase == 0:
            # 회전
            self.x = 0.0
            self.z = 2.0
            if (self.get_clock().now() - self.prevTime) > Duration(seconds = 1, nanoseconds=250_000_000):
                self.prevTime = self.get_clock().now()
                self.phase = 1
        elif self.phase == 1:
            # 진직
            self.x = 1.0
            self.z = 0.0
            if (self.get_clock().now() - self.prevTime) > Duration(seconds=2):
                self.prevTime = self.get_clock().now()
                self.phase = 0
        self.get_logger().info(f"phase: {self.phase}")

    def pose_callback(self, msg: Pose):
        self.pose_x = msg.x
        self.pose_y = msg.y
        self.pose_theta = msg.theta

def main():
    rclpy.init()
    node = Tbot_move()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
