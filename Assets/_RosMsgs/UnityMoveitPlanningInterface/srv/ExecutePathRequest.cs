//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityMoveitPlanningInterface
{
    [Serializable]
    public class ExecutePathRequest : Message
    {
        public const string k_RosMessageName = "unity_moveit_planning_interface/ExecutePath";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg plannedTrajectory;

        public ExecutePathRequest()
        {
            this.plannedTrajectory = new Moveit.RobotTrajectoryMsg();
        }

        public ExecutePathRequest(Moveit.RobotTrajectoryMsg plannedTrajectory)
        {
            this.plannedTrajectory = plannedTrajectory;
        }

        public static ExecutePathRequest Deserialize(MessageDeserializer deserializer) => new ExecutePathRequest(deserializer);

        private ExecutePathRequest(MessageDeserializer deserializer)
        {
            this.plannedTrajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.plannedTrajectory);
        }

        public override string ToString()
        {
            return "ExecutePathRequest: " +
            "\nplannedTrajectory: " + plannedTrajectory.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
