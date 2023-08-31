//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.BuiltinInterfaces;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class CartesianTrajectoryPointMsg : Message
    {
        public const string k_RosMessageName = "moveit_msgs/CartesianTrajectoryPoint";
        public override string RosMessageName => k_RosMessageName;

        //  The definition of a cartesian point in a trajectory. Defines the cartesian state of the point and it's time,
        //  following the pattern of the JointTrajectory message
        public CartesianPointMsg point;
        public DurationMsg time_from_start;

        public CartesianTrajectoryPointMsg()
        {
            this.point = new CartesianPointMsg();
            this.time_from_start = new DurationMsg();
        }

        public CartesianTrajectoryPointMsg(CartesianPointMsg point, DurationMsg time_from_start)
        {
            this.point = point;
            this.time_from_start = time_from_start;
        }

        public static CartesianTrajectoryPointMsg Deserialize(MessageDeserializer deserializer) => new CartesianTrajectoryPointMsg(deserializer);

        private CartesianTrajectoryPointMsg(MessageDeserializer deserializer)
        {
            this.point = CartesianPointMsg.Deserialize(deserializer);
            this.time_from_start = DurationMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.point);
            serializer.Write(this.time_from_start);
        }

        public override string ToString()
        {
            return "CartesianTrajectoryPointMsg: " +
            "\npoint: " + point.ToString() +
            "\ntime_from_start: " + time_from_start.ToString();
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
