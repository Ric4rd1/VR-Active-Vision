import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from xarm_msgs.srv import MoveCartesian, MoveJoint
from xarm_msgs.srv import SetInt16ById, SetInt16

class xyzTrack(Node):
    def __init__(self):
        super().__init__('xyztrack_servo')
        self.get_logger().info('xyzTrack servo initialized')

        # Subscribe to the track topic
        self.pose_sub = self.create_subscription(Pose, 'headset_pose', self.pose_callback, 10)

        # Create variables
        self.home = [200.0, 0.0, 200.0, 3.14, -1.4, 0.0]
        self.first_pose = None
        self.first_pose_received = False
        self.emergency_stop = False
        self.latest_target_pose = self.home[:] # last headset target
        self.last_pose = self.home[:]          # last sent to robot
               
        # Define constants
        self.speed = float(80)
        #self.speed_joint = float(0.2)
        self.wait = False
        self.gain = 3.0 # Scaling headset to robot movement  


        # Initiation routine 
        self.init_robot()
        self.set_mode_0()
        future = self.send_pose(pose=self.home)
        rclpy.spin_until_future_complete(self, future)  # wait until move finishes

        if future.result() is not None:
            self.get_logger().info("Homed successfully")
        else:
            self.get_logger().error("Failed to move home")

        self.get_logger().info('Robot initialized')

        self.set_mode_1()

        self.get_logger().info("Robot ready to start")

        self.dt = 1.0/200.0  
        # Create a fixed 125 Hz timer to publish smoothed poses
        self.timer = self.create_timer(self.dt, self.update_loop)

        # Interpolation constant
        self.tau = 0.03  # smoothing time constant (seconds)
        #self.k = 1.0 - math.exp(-self.dt/self.tau)
        self.k = 0.02

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

        # Create a client for the /ufactory/set_state service
        self.set_state_client = self.create_client(SetInt16, '/ufactory/set_state')
        while not self.set_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /ufactory/set_state not available, waiting...')

        # For Mode 0 
        # Create a client for the /ufactory/set_position service
        self.client_0 = self.create_client(MoveCartesian, '/ufactory/set_position')
        while not self.client_0.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ufactory service available')

        # For Mode 1 (servo)
        # Create a client for the /ufactory/set_servo_cartesian service
        self.client = self.create_client(MoveCartesian, '/ufactory/set_servo_cartesian')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info('Ufactory service available')

    def set_mode_0(self):
        # Call the /ufactory/set_mode service
        set_mode_request = SetInt16.Request()
        set_mode_request.data = 0 
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set mode successful: MODE 0')
        else:
            self.get_logger().error('Failed to call /ufactory/set_mode')

        # Call the /ufactory/set_state service
        set_state_request = SetInt16.Request()
        set_state_request.data = 0
        future = self.set_state_client.call_async(set_state_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set state successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_state')

    def set_mode_1(self):
        # Call the /ufactory/set_mode service
        set_mode_request = SetInt16.Request()
        set_mode_request.data = 1 # Servo mode, high frequency
        future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set mode successful: MODE 1')
        else:
            self.get_logger().error('Failed to call /ufactory/set_mode')

        # Call the /ufactory/set_state service
        set_state_request = SetInt16.Request()
        set_state_request.data = 0
        future = self.set_state_client.call_async(set_state_request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Set state successful')
        else:
            self.get_logger().error('Failed to call /ufactory/set_state')

    # Cartesian pose for mode 1 (servo)
    def send_servo_pose(self, pose):
        req = MoveCartesian.Request()
        req.pose = pose
        future = self.client.call_async(req)
        return future

    # Cartesian pose for mode 0
    def send_pose(self, pose):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = True
        req.pose = pose
        future = self.client_0.call_async(req)
        return future
    
    def lerp(self, start, end, t):
        return start + t * (end - start)

    def pose_callback(self, msg):
        if not self.first_pose_received:
            self.first_pose_received = True
            self.first_pose = msg
            self.get_logger().info("First pose received, setting reference frame")
            return

        if self.emergency_stop:
            return

        # Compute X displacement (meters → mm)
        x_offset = (msg.position.x - self.first_pose.position.x) * 1000.0 
        x_offset /= self.gain  # Apply gain 
        x_offset = max(0.0, min(200.0, x_offset)) # Clamp to [0, 200 mm]

        # Compute Y displacement (meters → mm)
        y_offset = (msg.position.y - self.first_pose.position.y) * 1000
        y_offset /= self.gain  # Apply gain
        y_offset = max(-100.0, min(100.0, y_offset)) # Clamp to [-100, 100 mm]

        # Compute Z displacement (meters → mm)
        z_offset = (msg.position.z - self.first_pose.position.z) * 1000
        z_offset /= self.gain  # Apply gain
        z_offset = max(-100.0, min(100.0, z_offset)) # Clamp to [-100, 100 mm]

        # Build absolute pose (only X moves, Y/Z and orientation stay at home)
        self.latest_target_pose = [
            self.home[0] + x_offset,   # X updated
            self.home[1] + y_offset,   # Y updated
            self.home[2] + z_offset,   # Z stays
            self.home[3],              # RX stays
            self.home[4],              # RY stays
            self.home[5],              # RZ stays
        ]
        
        #self.get_logger().info(f"Moving to XYZ = {target_pose[0]:.1f}X {target_pose[1]:.1f}Y {target_pose[2]:.1f}Z mm")

    def update_loop(self):
        if not self.first_pose_received or self.emergency_stop:
            return

        # Smoothly move last_pose towards latest_target_pose
        smoothed = []
        for lp, tp in zip(self.last_pose, self.latest_target_pose):
            smoothed.append(self.lerp(lp, tp, self.k))

        # Safety check: large jump
        d = ((self.latest_target_pose[0]-self.last_pose[0])**2 + (self.latest_target_pose[1]-self.last_pose[1])**2 + (self.latest_target_pose[2]-self.last_pose[2])**2)**0.5
        if abs(d) > 100.0:
            self.get_logger().warning(f"Large jump {d:.1f} mm, stopping")
            self.emergency_stop = True
            return

        self.send_servo_pose(smoothed)
        self.last_pose = smoothed

def main(args=None):
    rclpy.init(args=args)
    xyztrack = xyzTrack()
    rclpy.spin(xyztrack)
    xyztrack.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

