import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class AdaptiveOdometryCalculator(Node):
    def __init__(self):
        super().__init__('adaptive_odom_calculator')

        # 퍼블리셔와 구독자 설정
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.yaw_subscriber = self.create_subscription(Float32, '/filtered_yaw', self.yaw_callback, 10)  # Float32로 수정
        self.angular_velocity_subscriber = self.create_subscription(Float32, '/imu/angular_velocity', self.angular_velocity_callback, 10)  # 토픽 이름 수정

        # Odometry 계산 변수 초기화
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # 라디안 단위
        self.angular_velocity = 0.0
        self.last_time = self.get_clock().now()

        # 주기적으로 odometry 계산
        self.timer = self.create_timer(0.1, self.update_odometry)

    def yaw_callback(self, msg):
        yaw_degrees = msg.data  # Float32 타입 데이터를 바로 사용
        self.yaw = yaw_degrees * math.pi / 180  # degrees → radians
        self.get_logger().info(f"Received Yaw: {yaw_degrees:.2f} degrees, {self.yaw:.3f} radians")

    def angular_velocity_callback(self, msg):
        self.angular_velocity = msg.data
        self.get_logger().info(f"Received Angular Velocity Z: {self.angular_velocity:.3f} rad/s")

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # 초 단위
        self.last_time = current_time

        # Odometry 계산
        self.x += math.cos(self.yaw) * dt
        self.y += math.sin(self.yaw) * dt
        self.yaw += self.angular_velocity * dt

        # yaw 정규화
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        # odom 메시지 작성
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        odom.twist.twist.angular.z = self.angular_velocity

        # 퍼블리시
        self.odom_publisher.publish(odom)

        # 로그 출력
        self.get_logger().info(
            f"Odom: x={self.x:.3f}, y={self.y:.3f}, yaw(degrees)={math.degrees(self.yaw):.2f}, "
            f"yaw(radians)={self.yaw:.3f}, angular_velocity(z)={self.angular_velocity:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveOdometryCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
