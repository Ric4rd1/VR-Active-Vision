import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response 
import threading

class VideoStream(Node):
    def __init__(self):
        super().__init__('video_stream')
        self.get_logger().info('Video Stream Node Initialized')

        # Subscribe to video topic
        #self.image_sub = self.create_subscription(Image, '/camera/camera/color/image_rect_raw', self.image_callback, 10)
        self.image_sub = self.create_subscription(CompressedImage, '/camera/camera/color/image_rect_raw/compressed', self.image_callback, 10)

        self.bridge = CvBridge()
        self.img = None

        # Flask app for streaming
        self.app = Flask(__name__)
        @self.app.route('/')
        def video_feed():
            def generate():
                while True:
                    if self.img is not None:
                        ret, jpeg = cv2.imencode('.jpg', self.img)
                        frame = jpeg.tobytes()
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            return Response(generate(),
                            mimetype='multipart/x-mixed-replace; boundary=frame')
        # Start Flask app in a separate thread
        stream = threading.Thread(target=self.app.run, kwargs={'host':'0.0.0.0', 'port':5000, 'debug':True, 'use_reloader':False})
        stream.start()

    def image_callback(self, msg):
        try:
            #self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.img = self.bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
            #self.get_logger().info('Image received')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
        
def main(args=None):
    rclpy.init(args=args)
    video_stream_node = VideoStream()
    rclpy.spin(video_stream_node)
    video_stream_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


