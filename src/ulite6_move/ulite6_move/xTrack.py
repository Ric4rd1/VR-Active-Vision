import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import MoveCartesian, MoveJoint
from xarm_msgs.srv import SetInt16ById, SetInt16

class xTrack(Node):
    def __init__(self):
        super().__init__('xtrack')
        self.get_logger().info('xtrack initialized')

        # Subscribe to the track topic
        self.pose_sub = self.create_subscription(Pose, 'headset_pose', self.pose_callback, 10)

        # Create variables
        self.home = [200.0, 0.0, 200.0, 3.14, 30.0, 0.0]
        self.latest_pose = None
        self.first_pose = None
        self.first_pose_received = False
               
        # Define constants
        self.speed = float(80)
        self.speed_joint = float(0.2)
        self.wait = False
        self.gain = 5.0 # Scaling headset to robot movement  


        # Initiation routine 
        self.init_robot()
        self.send_request_cartesian(pose=self.home)
        self.get_logger().info('Robot initialized')

        # Timer for checking new poses (10 Hz)
        self.create_timer(0.05, self.timer_callback)
        
        self.get_logger().info("Robot ready to start")

    def init_robot(self):
        # Create a client for the /ufactory/motion_enable service
        self.motion_enable_client = self.create_client(SetInt16ById, '/ufactory/motion_enable')
        while not self.motion_enable_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/motion_enable not available, waiting...')

        # Call the /ufactory/motion_enable service
        motion_enable_request = SetInt16ById.Request()
        motion_enable_request.id = 8
        motion_enable_request.data = 1
        future = self.motion_enable_client.call_async(motion_enable_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Motion enable successful')
        else:
            self.get_logger().error('Failed to call /ufactory/motion_enable')

        # Create a client for the /ufactory/set_mode service
        self.set_mode_client = self.create_client(SetInt16, '/ufactory/set_mode')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/set_mode not available, waiting...')

        # Call the /ufactory/set_mode service
        set_mode_request = SetInt16.Request()
        set_mode_request.data = 0
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set mode successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_mode')

        # Create a client for the /ufactory/set_state service
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/set_state not available, waiting...')

        # Call the /ufactory/set_state service
        set_state_request = SetInt16.Request()
        set_state_request.data = 0
        future = self.set_state_client.call_async(set_state_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set state successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_state')

        # Create a client for the /ufactory/set_position service
        self.client = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ufactory service available')

        # Create a client for the /ufactory/set_servo_angles service
        self.client_joint = self.create_client(MoveJoint, '/ufactory/set_servo_angle')
        while not self.client_joint.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ufactory Joint service available')

    def send_request_cartesian(self, pose):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        req.pose = pose
        future = self.client.call_async(req)
        return future

    def pose_callback(self, msg):
        if not self.first_pose_received:
            self.first_pose_received = True
            self.first_pose = msg
            self.get_logger().info("First pose received, setting reference frame")
            return

        # Compute X displacement (meters → mm)
        x_offset = (msg.position.x - self.first_pose.position.x) * 1000.0 
        #x_offset /= self.gain  # Apply gain 
        #x_offset = max(0.0, min(200.0, x_offset)) # Clamp to [0, 200 mm]

        # Build absolute pose (only X moves, Y/Z and orientation stay at home)
        new_pose = [
            self.home[0] + x_offset,   # X updated
            self.home[1],              # Y stays
            self.home[2],              # Z stays
            self.home[3],              # RX stays
            self.home[4],              # RY stays
            self.home[5],              # RZ stays
        ]

        new_pose[0] = max(self.home[0] - 200.0, min(self.home[0] + 200.0, new_pose[0]))

        # Store the latest pose
        self.latest_pose = new_pose

        
    def timer_callback(self):
        if self.latest_pose is not None:
            self.send_request_cartesian(self.latest_pose)
            self.get_logger().info(f"Moving to X = {self.latest_pose[0]:.1f} mm")
            self.latest_pose = None  # reset to avoid repeating the same command


def main(args=None):
    rclpy.init(args=args)
    xtrack = xTrack()
    rclpy.spin(xtrack)
    xtrack.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

