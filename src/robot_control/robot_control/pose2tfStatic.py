import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

class Pose2tf(Node):
    def __init__(self):
        super().__init__('pose2tf')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        pose_msg = Pose()
        pose_msg.position.x = 1.0
        pose_msg.position.y = 2.0
        pose_msg.position.z = 0.0

        pose_msg.orientation.x = 0.0
        pose_msg.orientation.y = 0.0
        pose_msg.orientation.z = 0.0
        pose_msg.orientation.w = 1.0  # identity quaternion
        # publish static transform one at startup
        self.make_transforms(pose_msg)


    def make_transforms(self, pose_transform: Pose):
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "headset"

        # Transform
        t.transform.translation.x = pose_transform.position.x
        t.transform.translation.y = pose_transform.position.y
        t.transform.translation.z = pose_transform.position.z
        t.transform.rotation = pose_transform.orientation

        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform from world to headset: {t}")


def main(args=None):
    rclpy.init(args=args)
    pose2tf = Pose2tf()

    try:
        rclpy.spin(pose2tf)
    except KeyboardInterrupt:
        pass
    finally:
        pose2tf.destroy_node()
        if rclpy.ok():   # <-- Only call shutdown if still active
            rclpy.shutdown()

if __name__ == '__main__':
    main()