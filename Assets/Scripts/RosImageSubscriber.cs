using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;  // for ImageMsg

public class RosImageSubscriber : MonoBehaviour
{
    // ROS connector
    ROSConnection ros;

    // Topic to subscribe
    public string topicName = "/image_raw/compressed";

    // Material or UI element where we’ll show the image
    public Renderer targetRenderer; // assign in Inspector
    private Texture2D tex;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<CompressedImageMsg>(topicName, ImageCallback);

        // Create an empty texture to fill in later
        tex = new Texture2D(2, 2, TextureFormat.RGB24, false);
    }

    void ImageCallback(CompressedImageMsg msg)
    {
        // msg.data is a JPEG/PNG depending on ROS side
        tex.LoadImage(msg.data);

        // Apply to a material (e.g., on a Quad in VR space)
        if (targetRenderer != null)
        {
            targetRenderer.material.mainTexture = tex;
        }
    }
}
