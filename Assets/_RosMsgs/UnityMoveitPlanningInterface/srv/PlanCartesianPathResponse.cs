//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.UnityMoveitPlanningInterface
{
    [Serializable]
    public class PlanCartesianPathResponse : Message
    {
        public const string k_RosMessageName = "unity_moveit_planning_interface/PlanCartesianPath";
        public override string RosMessageName => k_RosMessageName;

        public Moveit.RobotTrajectoryMsg plannedTrajectory;

        public PlanCartesianPathResponse()
        {
            this.plannedTrajectory = new Moveit.RobotTrajectoryMsg();
        }

        public PlanCartesianPathResponse(Moveit.RobotTrajectoryMsg plannedTrajectory)
        {
            this.plannedTrajectory = plannedTrajectory;
        }

        public static PlanCartesianPathResponse Deserialize(MessageDeserializer deserializer) => new PlanCartesianPathResponse(deserializer);

        private PlanCartesianPathResponse(MessageDeserializer deserializer)
        {
            this.plannedTrajectory = Moveit.RobotTrajectoryMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.plannedTrajectory);
        }

        public override string ToString()
        {
            return "PlanCartesianPathResponse: " +
            "\nplannedTrajectory: " + plannedTrajectory.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize, MessageSubtopic.Response);
        }
    }
}
