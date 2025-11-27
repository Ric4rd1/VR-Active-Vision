import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2
from rclpy.qos import qos_profile_sensor_data


class YoloInference(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.model = YOLO('/home/ricard/unity_ws/src/ulite6_move/models/best.pt')  # Load the YOLOv8 model
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, 'camera/camera/color/image_rect_raw', self.camera_callback, 10) # For Puzzlebot
        self.pub = self.create_publisher(String, 'prediction', 10) 
        self.yolo_pub = self.create_publisher(Image, 'predict_image', qos_profile_sensor_data)
        
        self.img = None
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #cv2.GausssianBlur(self.img, (15, 15), 0, self.img)  # Apply Gaussian blur to the image
            self.valid_img = True
            #self.get_logger().info('Image received')
        except:
            self.get_logger().info('Failed to get an image')

    def timer_callback(self):
        if self.img is None:
            return

        # Publish image for debugging
        #debug_msg = self.bridge.cv2_to_imgmsg(self.img, "bgr8")
        #self.debug_pub.publish(debug_msg)

        results = self.model(self.img)
        frame = results[0].plot()
        self.yolo_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))




def main(args=None):
    rclpy.init(args=args)
    y_i = YoloInference()
    rclpy.spin(y_i)
    y_i.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()