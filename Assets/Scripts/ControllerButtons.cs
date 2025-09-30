using TMPro;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.XR;

public class ControllerButtons : MonoBehaviour
{
    public string originalScene = "GreenHouse";
    public float holdTime = 2f; // seconds to hold button

    private float rightButtonHeld = 0f;
    private float leftButtonHeld = 0f;

    void Start()
    {

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
    }
}
