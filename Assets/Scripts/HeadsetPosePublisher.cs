using RosMessageTypes.Geometry; // for PoseMsg or PoseStampedMsg
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;

public class HeadsetPosePublisher : MonoBehaviour
{
    public string topicName = "headset_pose";
    public float publishRate = 10f; // Hz
    private float publishTimer;
    private float delayTimer;

    private ROSConnection ros;

    // Transform of the headset (from XR Rig or camera)
    public Transform headsetTransform;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseMsg>(topicName);
        ros.RegisterPublisher<BoolMsg>("start_signal");
        publishTimer = 0f;
        delayTimer = 0f;

        // Send start signal
        BoolMsg msg = new BoolMsg(true);
        ros.Publish("start_signal", msg);
        Debug.Log("Start signal sent to ROS2");
    }

    void Update()
    {
        delayTimer += Time.deltaTime;

        // Only start publishing after 1 second delay
        if (delayTimer >= 1f)
        {
            publishTimer += Time.deltaTime;

            if (publishTimer > 1f / publishRate)
            {
                PublishHeadsetPose();
                publishTimer = 0f;
            }
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
