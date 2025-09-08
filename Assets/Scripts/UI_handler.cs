using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;  // <-- use standard messages

using TMPro; // Only if using TextMeshPro

public class UI_handler : MonoBehaviour
{
    ROSConnection ros;

    public TMP_InputField inputField; // Input field 


    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("chatter");
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnButtonPressed()
    {
        Debug.Log("Button was pressed!");
        string inputFieldText = inputField.text;
        StringMsg msg = new StringMsg($"From Unity: 'Send Message' button was pressed, with text: {inputFieldText}");
        ros.Publish("chatter", msg);
        Debug.Log("Sent: " + msg.data);

    }

    // This method will be called with a string parameter
    public void OnButtonPressedWithText(string inputText)
    {
        Debug.Log("Button pressed! Input: " + inputText);
        // Do whatever you want with inputText
    }
}
