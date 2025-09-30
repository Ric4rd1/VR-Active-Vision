using RosMessageTypes.Std; 
using System.Collections;
using System.Collections.Generic;
using TMPro; // Only if using TextMeshPro
using Unity.Robotics.ROSTCPConnector;
using UnityEngine;
using UnityEngine.SceneManagement;

public class UI_handler : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "start_signal";
    public string teleOpScene = "Teleop"; 

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

    public void OnStartButton()
    {
        Debug.Log("Start button pressed!");
        try
        {
            BoolMsg msg = new BoolMsg(true);
            ros.Publish(topicName, msg);
            Debug.Log("Start signal sent to ROS2");
        }
        catch (System.Exception e)
        {
            Debug.LogWarning("Failed to send ROS message: " + e.Message);
        }

        // Always load the next scene
        SceneManager.LoadScene(teleOpScene);
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
