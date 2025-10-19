using RosMessageTypes.Std;
using TMPro;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR;

public class ControllerButtons : MonoBehaviour
{
    public string originalScene = "GreenHouse";
    public float holdTime = 2f; // seconds to hold button b
    public float triggerCooldown = 1f; // seconds between trigger messages
    private float lastTriggerTime = -Mathf.Infinity; // last time trigger was pressed

    private AudioSource audioSource;


    private float rightButtonHeld = 0f;
    private float leftButtonHeld = 0f;

    ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<BoolMsg>("screenshot");

        audioSource = GetComponent<AudioSource>();

    }

    void Update()
    {
        // Right controller (B)
        InputDevice rightHand = InputDevices.GetDeviceAtXRNode(XRNode.RightHand);
        if (rightHand.TryGetFeatureValue(CommonUsages.secondaryButton, out bool rightPressed))
        {
            if (rightPressed)
            {
                rightButtonHeld += Time.deltaTime;
                if (rightButtonHeld >= holdTime)
                {
                    Debug.Log("Right controller B held -> switching scene");
                    SceneManager.LoadScene(originalScene);
                }
            }
            else
            {
                rightButtonHeld = 0f; // reset if released early
            }
        }



        // Left controller (Y)
        InputDevice leftHand = InputDevices.GetDeviceAtXRNode(XRNode.LeftHand);
        if (leftHand.TryGetFeatureValue(CommonUsages.secondaryButton, out bool leftPressed))
        {
            if (leftPressed)
            {
                leftButtonHeld += Time.deltaTime;
                if (leftButtonHeld >= holdTime)
                {
                    Debug.Log("Left controller Y held -> switching scene");
                    SceneManager.LoadScene(originalScene);
                }
            }
            else
            {
                leftButtonHeld = 0f;
            }
        }

        // === Right Trigger to send BoolMsg ===
        if (rightHand.TryGetFeatureValue(CommonUsages.triggerButton, out bool triggerPressed))
        {
            if (triggerPressed && Time.time - lastTriggerTime >= triggerCooldown)
            {
                BoolMsg msg = new BoolMsg(true);
                ros.Publish("screenshot", msg);
                lastTriggerTime = Time.time; // reset cooldown timer
                Debug.Log("Right trigger pressed -> Sent BoolMsg(true)");

                if (audioSource != null)
                    audioSource.Play();
            }
        }

    }
}
