import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')
        self.subscription = self.create_subscription(
            Odometry,
            '/vrpn_client_node/P1/pose',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            Odometry,
            'robot_pose',
            10
        )

    def odom_callback(self, msg):
        # Publish the received Odometry message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    odom_publisher = OdomPublisher()
    rclpy.spin(odom_publisher)
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()