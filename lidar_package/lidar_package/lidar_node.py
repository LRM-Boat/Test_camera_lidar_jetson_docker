import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # предотвращаем предупреждение об "unused variable"

    def listener_callback(self, msg):
        self.get_logger().info(f'Received Lidar data: {msg.ranges[:5]}')  # Вывод первых 5 значений диапазона


def main(args=None):
    rclpy.init(args=args)
    lidar_node = LidarNode()

    try:
        rclpy.spin(lidar_node)
    except KeyboardInterrupt:
        pass

    lidar_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
