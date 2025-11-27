import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import json

class LatencyTester(Node):
    def __init__(self):
        super().__init__('latency_tester')

        # Publisher to send ping messages to Unity
        self.publisher_ = self.create_publisher(String, '/latency_test', 10)
        # Subscriber to receive response from Unity
        self.subscription = self.create_subscription(
            String,
            '/latency_response',
            self.listener_callback,
            10)
        
        self.timer = self.create_timer(1.0, self.send_ping)  # send every second
        self.sent_timestamps = {}  # store send times
        self.seq = 0

    def send_ping(self):
        """Send a timestamped message to Unity"""
        msg = String()
        timestamp = time.time()
        msg.data = json.dumps({'seq': self.seq, 'timestamp': timestamp})
        self.publisher_.publish(msg)
        self.sent_timestamps[self.seq] = timestamp
        self.get_logger().info(f'Ping {self.seq} sent at {timestamp:.6f}')
        self.seq += 1

    def listener_callback(self, msg):
        """Compute latency when Unity echoes back"""
        try:
            data = json.loads(msg.data)
            seq = data['seq']
            if seq in self.sent_timestamps:
                sent_time = self.sent_timestamps.pop(seq)
                now = time.time()
                latency = (now - sent_time) * 1000.0  # ms
                self.get_logger().info(f'Latency for seq {seq}: {latency:.2f} ms')
        except Exception as e:
            self.get_logger().error(f'Error parsing message: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = LatencyTester()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
