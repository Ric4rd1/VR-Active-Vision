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
    public string teleOpScene = "Teleop"; 

    public TMP_InputField inputField; // Input field
    public TMP_Dropdown dropdown; 


    // Start is called before the first frame update
    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<StringMsg>("chatter");
        ros.RegisterPublisher<StringMsg>("teleop_config");

        ros.Publish("teleop_config", new StringMsg("MODE,STANDING")); // Default mode
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    public void OnDropdownConfirm()
    {
        int option = dropdown.value; // The index of the selected option
        string selectedText = dropdown.options[option].text; // The actual text of the option

        Debug.Log("Dropdown selected: " + selectedText);

        string msgText;
        switch (option)
        {
            case 0:
                msgText = "MODE,STANDING";
                break;
            case 1:
                msgText = "MODE,SITTING";
                break;
            default:
                msgText = "Unknown option";
                break;
        }

        // Send message to ROS
        StringMsg msg = new StringMsg(msgText);
        ros.Publish("teleop_config", msg);
        Debug.Log("Sent: " + msg.data);
    }


    public void OnStartButton()
    {   
        SceneManager.LoadScene(teleOpScene); // Switch to teleop scene

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
