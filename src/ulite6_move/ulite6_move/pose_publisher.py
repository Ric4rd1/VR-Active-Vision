import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import threading

class PosePublisherConsole(Node):
    def __init__(self):
        super().__init__('pose_simulator_console')
        self.publisher_ = self.create_publisher(Pose, 'headset_pose', 10)

        # Current X value in mm
        self.x_value = 0.0

        # Start thread to handle user input
        threading.Thread(target=self.user_input_thread, daemon=True).start()

        # Publish timer (10 Hz)
        self.create_timer(0.1, self.publish_pose)

    def user_input_thread(self):
        print("Console Pose Simulator (X-axis, mm)")
        print("Type a number between 0 and 200 to update X position. Type 'q' to quit.")
        while True:
            user_input = input("New X value (mm): ")
            if user_input.lower() == 'q':
                print("Exiting input thread...")
                break
            try:
                x = float(user_input)
                if 0.0 <= x <= 2000.0:
                    self.x_value = x
                    print(f"Set X to {x:.1f} mm")
                else:
                    print("Value out of range (0 - 200 mm)")
            except ValueError:
                print("Invalid input, enter a number.")

    def publish_pose(self):
        msg = Pose()
        # Convert mm to meters for the Pose message
        msg.position.x = self.x_value / 1000.0
        msg.position.y = 0.0
        msg.position.z = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published X = {self.x_value:.1f} mm")


def main(args=None):
    rclpy.init(args=args)
    node = PosePublisherConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
