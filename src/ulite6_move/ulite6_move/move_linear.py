import rclpy
from rclpy.node import Node
from xarm_msgs.srv import MoveCartesian, MoveJoint
from xarm_msgs.srv import SetInt16ById, SetInt16, Call
import time

class move_line(Node):
    def __init__(self):
        super().__init__('Move_line')
        self.get_logger().info('Move_line initialized')

        # Create variables
        self.target = None
        self.x = 0
        self.y = 0
        self.z = 0
        self.home = [250.0, 0.0, 200.0, 3.14, 30.0, 0.0]
        self.pose = self.home
               
        # Define constants
        self.call = Call.Request()
        self.speed = float(80)
        self.speed_joint = float(0.2)
        self.wait = True


        # Initiation routine 
        self.init_robot()
        self.get_logger().info('Robot initialized')
        
        # Move to home position and open gripper
        self.send_request_cartesian(pose=self.home)
        self.open_gripper()
        self.stop_gripper()
        
        self.get_logger().info("Robot ready to start")
        self.square_routine()

        

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

        # Create a client for the /ufactory/close_lite6_gripper service 
        self.close_gripper_client = self.create_client(Call, '/ufactory/close_lite6_gripper')
        while not self.close_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Close Gripper service not available, waiting again...')
        self.get_logger().info('Close Gripper service available')

        # Create a client for the /ufactory/open_lite6_gripper service
        self.open_gripper_client = self.create_client(Call, '/ufactory/open_lite6_gripper')
        while not self.open_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Open Gripper service not available, waiting again...')
        self.get_logger().info('Open Gripper service available')

        # Create a client for the /ufactory/stop_lite6_gripper service
        self.stop_gripper_client = self.create_client(Call, '/ufactory/stop_lite6_gripper')
        while not self.stop_gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Stop Gripper service not available, waiting again...')
        self.get_logger().info('Stop Gripper service available')

    def close_gripper(self):
        self.future = self.close_gripper_client.call_async(self.call)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Gripper closed")
        time.sleep(2)
    
    def open_gripper(self): 
        self.future = self.open_gripper_client.call_async(self.call)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Gripper opened")
        time.sleep(2)

    def stop_gripper(self):
        self.future = self.stop_gripper_client.call_async(self.call)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("Gripper stopped")

    def send_request_cartesian(self, pose = None):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        if pose is None:
            req.pose = self.pose
        else: 
            req.pose = pose
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def send_request_relative_cartesian(self, pose = None):
        req = MoveCartesian.Request()
        req.speed = self.speed
        req.wait = self.wait
        req.relative = True
        if pose is None:
            req.pose = self.pose
        else: 
            req.pose = pose
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
    
    def send_request_joint(self, angles):
        req = MoveJoint.Request()
        req.angles = angles        # full 6-element list of joint angles
        req.speed = self.speed_joint
        req.acc = 100.0              # you can tune this
        req.wait = self.wait
        req.relative = True       # relative move 
        future = self.client_joint.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Joint move executed: {angles}")
        else:
            self.get_logger().error("Failed to move joints")


    
    def square_routine(self):
        first_position = [300.0, 50.0, 200.0, 3.14, 30.0, 0.0]
        second_position = [400.0, 50.0, 200.0, 3.14, 30.0, 0.0]
        third_position = [300.0, 50.0, 200.0, 3.14, 30.0, 0.0]
        fourth_position = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
        fifth_position = [0.0, 0.0, 100.0, 0.0, 0.0, 0.0]
        self.send_request_cartesian(pose=first_position)
        self.send_request_cartesian(pose=second_position)
        self.send_request_cartesian(pose=third_position)
        self.send_request_joint(angles=fourth_position)
        self.send_request_relative_cartesian(pose=fifth_position)
        


def main(args=None):
    # Required lines for any node
    rclpy.init(args=args)
    node = move_line()
    rclpy.spin(node)
    # Optional but good practices
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()