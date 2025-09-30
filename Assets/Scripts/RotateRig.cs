using UnityEngine;

public class RotateRig : MonoBehaviour
{
    void Start()
    {
        // Rotate 180 degrees around Y-axis
        transform.Rotate(0, 180f, 0);
    }
}
