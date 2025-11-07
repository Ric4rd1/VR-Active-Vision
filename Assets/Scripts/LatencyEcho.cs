using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;

public class LatencyEcho : MonoBehaviour
{
    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("/latency_response");
        ros.Subscribe<StringMsg>("/latency_test", OnPingReceived);
    }

    void OnPingReceived(StringMsg msg)
    {
        // Echo the same message back
        ros.Publish("/latency_response", msg);
    }
}
