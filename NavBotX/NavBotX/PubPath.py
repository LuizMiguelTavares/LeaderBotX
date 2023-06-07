import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Quaternion, Twist, Pose, Vector3
from math import cos, sin, pi

import csv
import os

class CircularPathPublisher(Node):
    def __init__(self):
        super().__init__('circular_path_publisher')

        self.radius = 1
        self.freq = 0.1
        T = 20
        self.angular_velocity = 2*pi/T

        self.start_time = None

        self.publisher = self.create_publisher(Odometry,
                                                'odom',
                                                10)
        
        self.subscription = self.create_subscription(
            Bool,
            'emergency_flag',
            self.emergency_button_callback,
            10 
        )
        
        # Create a CSV file to store the data
        # self.csv_file = open('circular_path_data.csv', 'w')
        # self.csv_writer = csv.writer(self.csv_file)
        # self.csv_writer.writerow(['Time', 'X', 'Y'])

        self.timer = self.create_timer(self.freq,
                                        self.publish_odometry)
        
        # Initializing emergency button to False
        self.btn_emergencia = False
        
    def publish_odometry(self):

        # Get the current timestamp
        current_time = self.get_clock().now().to_msg()

        if self.start_time is None:
            self.start_time = current_time

        # Calculate elapsed time
        elapsed_time = (current_time.sec - self.start_time.sec) + (current_time.nanosec - self.start_time.nanosec) / 1e9

        x = self.radius * cos(self.angular_velocity * elapsed_time)
        y = self.radius * sin(self.angular_velocity * elapsed_time)
        vx = -self.radius * self.angular_velocity * sin(self.angular_velocity * elapsed_time)
        vy = self.radius * self.angular_velocity * cos(self.angular_velocity * elapsed_time)

        # # Write the data to the CSV file
        # self.csv_writer.writerow([elapsed_time, x, y])

        # Create the Odometry message
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'odom'
        odometry.child_frame_id = 'base_link'
        odometry.pose.pose = Pose(position=Point(x=x, y=y, z=0.0), orientation=Quaternion())
        odometry.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odometry.twist.twist.angular = Vector3()

        self.publisher.publish(odometry)

    def emergency_button_callback(self, msg):
        if msg.data:
            self.btn_emergencia = True
            self.get_logger().info('Path publisher node stopping by Emergency')
            raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    circular_path_publisher = CircularPathPublisher()
    rclpy.spin(circular_path_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()