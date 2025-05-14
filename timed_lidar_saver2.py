import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D
import math
import csv
import time

class LidarAndPoseLogger(Node):
    def __init__(self, total_duration=10.0):
        super().__init__('lidar_and_pose_logger')

        # LiDAR subscriber
        self.subscription_lidar = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        # Pose subscriber
        self.subscription_pose = self.create_subscription(
            Pose2D,
            '/odom',
            self.pose_callback,
            10)

        self.total_duration = total_duration
        self.capture_interval = 1.0  # seconds
        self.start_time = time.time()
        self.next_capture_time = self.start_time
        self.capture_count = 0

        # Most recent pose
        self.latest_pose = Pose2D()

        # CSV files
        self.lidar_csv = open('lidar_captures.csv', 'w', newline='')
        self.lidar_writer = csv.writer(self.lidar_csv)
        self.lidar_writer.writerow(['capture', 'lidar_points'])

        self.pose_csv = open('pose_captures.csv', 'w', newline='')
        self.pose_writer = csv.writer(self.pose_csv)
        self.pose_writer.writerow(['capture', 'x', 'y', 'theta'])

        self.get_logger().info(f"Logging LiDAR and Pose data every {self.capture_interval}s for {total_duration}s.")

    def pose_callback(self, msg: Pose2D):
        # Save latest pose for synchronized logging
        self.latest_pose = msg

    def lidar_callback(self, msg: LaserScan):
        current_time = time.time()
        elapsed = current_time - self.start_time

        if elapsed > self.total_duration:
            self.get_logger().info("Finished logging. Shutting down.")
            self.lidar_csv.close()
            self.pose_csv.close()
            rclpy.shutdown()
            return

        if current_time >= self.next_capture_time:
            self.capture_count += 1
            self.next_capture_time += self.capture_interval

            # Convert LiDAR data to (x, y) points
            angle = msg.angle_min
            points = []
            for r in msg.ranges:
                if math.isfinite(r) and 0.1 < r < 30.0:
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append(f"({x},{y})")
                angle += msg.angle_increment

            point_string = ';'.join(points)
            self.lidar_writer.writerow([self.capture_count, point_string])

            # Write latest pose with the same capture index
            self.pose_writer.writerow([
                self.capture_count,
                self.latest_pose.x,
                self.latest_pose.y,
                self.latest_pose.theta
            ])

            self.get_logger().info(f"Captured #{self.capture_count}: LiDAR and Pose")

def main(args=None):
    rclpy.init(args=args)
    node = LidarAndPoseLogger(total_duration=20.0)  # Change duration as needed
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
