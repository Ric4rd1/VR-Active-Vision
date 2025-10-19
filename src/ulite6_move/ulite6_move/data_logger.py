import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import os

class DataLogger(Node):
    def __init__(self):
        super().__init__('data_logger')
        self.user_name = input("--> Enter user name: ")
        # Subscribe to start signal and image and screenshot topics
        self.start_sub = self.create_subscription(Bool, 'start_signal', self.start_callback, 10)
        self.image_sub = self.create_subscription(Image, 'camera/camera/color/image_rect_raw', self.image_callback, 10)
        self.ss_sub = self.create_subscription(Bool, 'screenshot', self.ss_callback, 10)
        
        self.bridge = CvBridge()
        self.img = None

        self.valid_img = False
        self.count = 0
        self.last_timestamp = 0.0  # store previous screenshot time
        self.start_timestamp = 0.0 # store start time

        self.user_dir = f'/home/ricard/unity_ws/src/ulite6_move/logs/{self.user_name.strip()}'
        os.makedirs(self.user_dir, exist_ok=True)
        self.log_file = os.path.join(self.user_dir, f"timestamps_{self.user_name}.txt")

        self.get_logger().info('Data Logger Node has been started.')

        # Initialize the log file with a header
        with open(self.log_file, 'w') as f:
            f.write("Screenshot#\tTimestamp(s)\tDelta_t(s)\n")


    def start_callback(self, msg):
        if msg.data:
            self.get_logger().info('Data logging started')
            self.start_timestamp = self.get_clock().now().nanoseconds * 1e-9  # Convert to seconds
        elif not msg.data:
            self.get_logger().info('Data logging stopped')
            with open(self.log_file, 'a') as f:
                f.write(f"Stopped at {self.get_clock().now().nanoseconds * 1e-9 - self.start_timestamp:.3f} seconds\n")

    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.valid_img = True
        except:
            self.get_logger().info('Failed to get an image')

    def ss_callback(self, msg):
        if msg.data and self.valid_img:
            timestamp = self.get_clock().now().nanoseconds * 1e-9 - self.start_timestamp # Convert to seconds
            
            self.count += 1

            filename = os.path.join(self.user_dir, f'#{self.count}.png')
            cv.imwrite(filename, self.img)

            delta_t = 0.0
            if self.last_timestamp is not None:
                delta_t = timestamp - self.last_timestamp
            self.last_timestamp = timestamp

            # Write to log file
            with open(self.log_file, 'a') as f:
                f.write(f"{self.count}\t{timestamp:.3f}\t{delta_t:.3f}\n")

            self.get_logger().info(f'Screenshot saved: {filename}')
        else:
            self.get_logger().info('No valid image to save screenshot')

def main(args=None):
    rclpy.init(args=args)
    data_logger = DataLogger()
    rclpy.spin(data_logger)
    data_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()