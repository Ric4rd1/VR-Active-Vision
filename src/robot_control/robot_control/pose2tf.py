import rclpy 
from rclpy.node import Node 
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class Pose2tf(Node):
    def __init__(self):
        super().__init__('pose2tf')

        self.pose_topic = "headset_pose"

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscription
        self.pose_sub = self.create_subscription(Pose, self.pose_topic, self.make_transforms, 10)

    def make_transforms(self, msg):
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "headset"

        # Transform
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)
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