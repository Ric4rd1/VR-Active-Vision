using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Geometry; // for PoseMsg or PoseStampedMsg

public class HeadsetPosePublisher : MonoBehaviour
{
    public string topicName = "headset_pose";
    public float publishRate = 10f; // Hz
    private float timer;

    private ROSConnection ros;

    // Transform of the headset (from XR Rig or camera)
    public Transform headsetTransform;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
        timer = 0f;
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer > 1f / publishRate)
        {
            PublishHeadsetPose();
            timer = 0f;
        }
    }

    void PublishHeadsetPose()
    {
        if (headsetTransform == null) return;

        Vector3 position = headsetTransform.position;
        Quaternion rotation = headsetTransform.rotation;

        // Convert to ROS FLU coordinate system
        position = PositionToFLU(position);
        rotation = RotationToFLU(rotation);

        PoseMsg msg = new PoseMsg();
        msg.position = new PointMsg(position.x, position.y, position.z);
        msg.orientation = new QuaternionMsg(rotation.x, rotation.y, rotation.z, rotation.w);

        ros.Publish(topicName, msg);
    }

    Vector3 PositionToFLU(Vector3 unityPosition)
    {
        // Unity (x-right, y-up, z-forward) -> ROS FLU (x-forward, y-left, z-up)
        return new Vector3(
            unityPosition.z,       // x_ros = z_unity
            -unityPosition.x,      // y_ros = -x_unity
            unityPosition.y        // z_ros = y_unity
        );
    }

    Quaternion RotationToFLU(Quaternion unityQuat)
    {
        // Swap axes to match ROS FLU
        return new Quaternion(
            unityQuat.z,    // x_ros = z_unity
            -unityQuat.x,   // y_ros = -x_unity
            unityQuat.y,    // z_ros = y_unity
            unityQuat.w     // w stays the same
        );
    }
}
