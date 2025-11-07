import rclpy
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, String
from xarm_msgs.srv import MoveCartesian, MoveJoint
from xarm_msgs.srv import SetInt16ById, SetInt16

class xyzrTrack(Node):
    def __init__(self):
        super().__init__('xyzrtrack_servo')
        self.get_logger().info('xyzrTrack servo initialized')

        # Subscribe to the track topic
        self.pose_sub = self.create_subscription(Pose, 'headset_pose', self.pose_callback, 10)
        # Subscribe to the start topic
        self.start_sub = self.create_subscription(Bool, 'start_signal', self.start_callback, 10)
        # Subscribe to config topic
        self.config_sub = self.create_subscription(String, 'teleop_config', self.config_callback, 10)

        # Debug publisher
        self.smoothed_pose_pub = self.create_publisher(Pose, 'smoothed_headset_pose', 10)

        # Create variables
        self.home = [250.0, 0.0, 425.0, 3.14, -1.4, 0.0]
        #self.home = [200.0, 0.0, 200.0, 3.14, -1.4, 0.0]
        self.start = False
        self.first_pose = None
        self.first_pose_received = False
        self.emergency_stop = False
        self.latest_target_pose = self.home[:] # last headset target
        self.last_pose = self.home[:]          # last sent to robot
        self.standing = False
        self.sitting = False
               
        # Define constants
        self.speed = float(80)
        #self.speed_joint = float(0.2)
        self.wait = False
        self.gain = 3.0 # Scaling headset to robot movement 
        self.sitting_gain = 0.8 # Scaling headset to robot movement 
        # Interpolation
        self.tau = 0.03  # smoothing time constant (seconds)
        #self.k = 1.0 - math.exp(-self.dt/self.tau)
        self.k = 0.02

        # Initiation routine 
        self.init_robot() # Initialize robot and services
        self.get_logger().info('Robot Configuration initialized')
        self.go_home() # Move to home position
        self.get_logger().info("Robot ready to start")

        self.dt = 1.0/200.0  
        # Create a fixed 125 Hz timer to publish smoothed poses
        self.timer = self.create_timer(self.dt, self.update_loop)

        

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

    def start_callback(self, msg):
        if msg.data and not self.start:
            self.get_logger().info("Start signal received, starting tracking")
            self.start = True
            self.emergency_stop = False
        elif not msg.data and self.start:
            '''
            self.get_logger().info("Stop signal received, stopping tracking")
            self.start = False
            self.emergency_stop = True
            self.first_pose_received = False
            '''
            self.first_pose_received = False

    def go_home(self):
        self.set_mode_0()
        future = self.send_pose(pose=self.home)
        rclpy.spin_until_future_complete(self, future)  # wait until move finishes

        if future.result() is not None:
            self.get_logger().info("Homed successfully")
        else:
            self.get_logger().error("Failed to move home")

        self.set_mode_1() # Set to mode 1 (servo)

        return future.result()

    def config_callback(self, msg):
        config = msg.data.split(',')
        if config[0] == 'MODE':
            if config[1] == 'STANDING':
                self.get_logger().info("Switching to STANDING MODE")
                self.first_pose_received = False
                self.standing = True
                self.sitting = False
                #self.go_home()
            elif config[1] == 'SITTING':
                self.get_logger().info("Switching to SITTING MODE")
                self.first_pose_received = False
                self.standing = False
                self.sitting = True
                #self.go_home()

        

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
    
    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        Roll  - rotation around X-axis
        Pitch - rotation around Y-axis
        Yaw   - rotation around Z-axis
        """

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90° if out of range
        else:
            pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def pose_callback(self, msg):
        if not self.start or self.emergency_stop:
            return
        
        if not self.first_pose_received:
            self.first_pose_received = True
            self.first_pose = msg
            self.first_pose_orientation = self.quaternion_to_euler(
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.orientation.w
            )
            self.get_logger().info("First pose received, setting reference frame")
            return
        
        if self.standing:
            self.standing_mode(msg)
        elif self.sitting:
            self.sitting_mode(msg)

    def standing_mode(self, msg):
        #self.get_logger().info("Standing mode active")
         # Compute X displacement (meters → mm)
        x_offset = (msg.position.x - self.first_pose.position.x) * 1000.0 
        x_offset /= self.gain  # Apply gain 
        x_offset = max(0.0, min(80.0, x_offset)) # Clamp to [0, 200 mm]

        # Compute Y displacement (meters → mm)
        y_offset = (msg.position.y - self.first_pose.position.y) * 1000
        y_offset /= self.gain  # Apply gain
        y_offset = max(-200.0, min(220.0, y_offset)) # Clamp to [-100, 100 mm]

        # Compute Z displacement (meters → mm)
        z_offset = (msg.position.z - self.first_pose.position.z) * 1000
        z_offset /= self.gain/5.0  # Apply gain
        z_offset = max(-250.0, min(150.0, z_offset)) # Clamp to [-100, 100 mm]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # Pitch offset (radians)
        pitch_offset = pitch - self.first_pose_orientation[1]
        pitch_offset = max(-math.radians(20), min(math.radians(20), pitch_offset)) 
        # Yaw offset (radians)
        yaw_offset = yaw - self.first_pose_orientation[2]
        yaw_offset = max(-math.radians(45), min(math.radians(45), yaw_offset)) 
        # Build absolute pose (only X moves, Y/Z and orientation stay at home)
        self.latest_target_pose = [
            self.home[0] + x_offset,      # X updated
            self.home[1] + y_offset,      # Y updated
            self.home[2] + z_offset,      # Z stays
            self.home[3],                 # RX stays
            self.home[4] - pitch_offset,  # RY stays
            self.home[5] - yaw_offset     # RZ stays
        ]
        
        #self.get_logger().info(f"Moving to XYZ = {target_pose[0]:.1f}X {target_pose[1]:.1f}Y {target_pose[2]:.1f}Z mm")

    def sitting_mode(self, msg):
        #self.get_logger().info("Sitting mode active")
         # Compute X displacement (meters → mm)
        x_offset = (msg.position.x - self.first_pose.position.x) * 1000.0 
        x_offset /= self.sitting_gain  # Apply gain 
        x_offset = max(0.0, min(200.0, x_offset)) # Clamp to [0, 200 mm]

        # Compute Y displacement (meters → mm)
        y_offset = (msg.position.y - self.first_pose.position.y) * 1000
        y_offset /= self.sitting_gain  # Apply gain
        y_offset = max(-100.0, min(100.0, y_offset)) # Clamp to [-100, 100 mm]

        # Compute Z displacement (meters → mm)
        z_offset = (msg.position.z - self.first_pose.position.z) * 1000
        z_offset /= self.sitting_gain/2.0  # Apply gain
        z_offset = max(-100.0, min(100.0, z_offset)) # Clamp to [-100, 100 mm]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        # Pitch offset (radians)
        pitch_offset = pitch - self.first_pose_orientation[1]
        pitch_offset = max(-math.radians(20), min(math.radians(20), pitch_offset)) 
        # Yaw offset (radians)
        yaw_offset = yaw - self.first_pose_orientation[2]
        yaw_offset = max(-math.radians(45), min(math.radians(45), yaw_offset)) 
        # Build absolute pose (only X moves, Y/Z and orientation stay at home)
        self.latest_target_pose = [
            self.home[0] + x_offset,      # X updated
            self.home[1] + y_offset,      # Y updated
            self.home[2] + z_offset,      # Z stays
            self.home[3],                 # RX stays
            self.home[4] - pitch_offset,  # RY stays
            self.home[5] - yaw_offset     # RZ stays
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
        if abs(d) > 251.0:
            self.get_logger().warning(f"Large jump {d:.1f} mm, stopping")
            self.emergency_stop = True
            return
        
        # Publish smoothed pose for debugging
        '''
        pose_msg = Pose()
        pose_msg.position.x = smoothed[0] / 1000.0 
        pose_msg.position.y = (smoothed[1] / 1000.0) * self.gain + self.first_pose.position.y
        pose_msg.position.z = smoothed[2] / 1000.0 
        self.smoothed_pose_pub.publish(pose_msg)
        '''
        self.send_servo_pose(smoothed)
        self.last_pose = smoothed

def main(args=None):
    rclpy.init(args=args)
    xyzrtrack = xyzrTrack()
    rclpy.spin(xyzrtrack)
    xyzrtrack.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
        

