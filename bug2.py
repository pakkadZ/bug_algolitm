import rclpy
from rclpy.node import Node  # นำเข้า Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class TurtleBotBug2(Node):
    def __init__(self):
        super().__init__('turtlebot_bug2')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd = Twist()

        self.forward_speed = 0.2
        self.turn_speed = 0.5
        self.safe_distance = 0.4
        self.following_wall = False
        self.distance_travelled = 0.0
        self.target_x = 1.8  # เป้าหมายตำแหน่ง X คือ 1.8 เมตร
        self.last_time = self.get_clock().now()

        self.current_position = None  # สถานะตำแหน่งปัจจุบัน

    def lidar_callback(self, msg):
        front_distance = min(min(msg.ranges[0:30] + msg.ranges[-30:]), 10)
        left_distance = min(min(msg.ranges[30:90]), 10)
        right_distance = min(min(msg.ranges[-90:-30]), 10)

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        self.distance_travelled += self.forward_speed * dt

        if self.current_position and self.current_position.x >= self.target_x:
            # หยุดหุ่นยนต์เมื่อถึงตำแหน่ง X ที่กำหนด
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info("หุ่นยนต์ถึงจุดหมาย x >= 1.8 เมตรแล้ว!")
            return

        if front_distance < self.safe_distance:
            self.following_wall = True
            if left_distance > right_distance:
                self.cmd.angular.z = self.turn_speed
            else:
                self.cmd.angular.z = -self.turn_speed
            self.cmd.linear.x = 0.0
        elif self.following_wall and front_distance > self.safe_distance:
            self.following_wall = False
            self.cmd.angular.z = 0.0
        else:
            self.cmd.linear.x = self.forward_speed

        self.publisher_.publish(self.cmd)

    def odom_callback(self, msg):
        # รับข้อมูลตำแหน่งจาก Odometry
        position = msg.pose.pose.position
        self.current_position = position
        self.get_logger().info(f"ตำแหน่งปัจจุบัน: x={position.x} m, y={position.y} m, z={position.z} m")

        # ตรวจสอบว่า x >= 1.8 หรือไม่
        if position.x >= self.target_x:
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.publisher_.publish(self.cmd)
            self.get_logger().info("หุ่นยนต์ถึงจุดหมาย x >= 1.8 เมตรแล้ว!")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotBug2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
