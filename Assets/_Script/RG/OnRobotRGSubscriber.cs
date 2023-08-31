using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.OnrobotRgControl;

namespace Telepatia.URControl
{
    public class OnRobotRGSubscriber : MonoBehaviour
    {
        void Start()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<OnRobotRGInputMsg>("OnRobotRGInput", MirrorRG2);
        }

        void MirrorRG2(OnRobotRGInputMsg message)
        {
            float offset = 92;
            float rawWidth = message.gGWD;
            GetComponent<OnRobotRGGripperController>().width = rawWidth - offset;
        }
    }
}