using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.OnrobotRgControl;

namespace Telepatia.URControl
{
    public class OnRobotRGServiceClient : MonoBehaviour
    {
        ROSConnection ros;
        public string serviceName = "onrobot_rg/set_command";


        float awaitingResponseUntilTimestamp = -1;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterRosService<OnRobotRGCommandRequest, OnRobotRGCommandResponse>(serviceName);
        }

        private void Update()
        {
            ushort w = (ushort)GetComponent<OnRobotRGGripperController>().width;
            OnRobotRGCommandRequest gripperServiceRequest = new OnRobotRGCommandRequest(w, 200);
            if (Time.time > awaitingResponseUntilTimestamp)
            {
                ros.SendServiceMessage<OnRobotRGCommandResponse>(serviceName, gripperServiceRequest, CallbackDestination);
                awaitingResponseUntilTimestamp = Time.time + 1.0f; // don't send again for 1 second, or until we receive a response
            }
        }

        void CallbackDestination(OnRobotRGCommandResponse response)
        {
            awaitingResponseUntilTimestamp = -1;
        }

        // public void GripperSetCommand()
        // {
        //     ushort w = (ushort)GetComponent<OnRobotRGGripperController>().width;
        //     OnRobotRGCommandRequest gripperServiceRequest = new OnRobotRGCommandRequest(w, 200);
        //     ros.SendServiceMessage<OnRobotRGCommandResponse>(serviceName, gripperServiceRequest, CallbackDestination);
        // }
    }
}