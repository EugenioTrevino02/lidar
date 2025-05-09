import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import csv
import time

class LidarCsvLogger(Node):
    def __init__(self, total_duration=10.0):
        super().__init__('lidar_csv_logger')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        
        self.total_duration = total_duration
        self.capture_interval = 1.0  # Capture every second
        self.start_time = time.time()
        self.next_capture_time = self.start_time
        self.capture_count = 0

        self.csv_file = open('lidar_captures.csv', 'w', newline='')
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(['capture', 'lidar_points'])  # CSV header

        self.get_logger().info(f"Logging LiDAR data every second for {total_duration} seconds.")

    def listener_callback(self, msg):
        current_time = time.time()
        elapsed = current_time - self.start_time

        if elapsed > self.total_duration:
            self.get_logger().info("Finished logging. Shutting down.")
            self.csv_file.close()
            rclpy.shutdown()
            return

        # Only save once per second
        if current_time >= self.next_capture_time:
            self.capture_count += 1
            self.next_capture_time += self.capture_interval

            # Convert ranges to (x, y) pairs
            angle = msg.angle_min
            points = []
            for r in msg.ranges:
                if math.isfinite(r) and 0.1 < r < 30.0:  # Filter out invalid values
                    x = r * math.cos(angle)
                    y = r * math.sin(angle)
                    points.append(f"({x:.1f}, {y:.1f})")
                angle += msg.angle_increment

            point_string = ';'.join(points)
            self.writer.writerow([self.capture_count, point_string])
            self.get_logger().info(f"Captured {self.capture_count}")

def main(args=None):
    rclpy.init(args=args)
    node = LidarCsvLogger(total_duration=20.0)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

