//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class PlanningSceneWorldMsg : Message
    {
        public const string k_RosMessageName = "moveit_msgs/PlanningSceneWorld";
        public override string RosMessageName => k_RosMessageName;

        //  collision objects
        public CollisionObjectMsg[] collision_objects;
        //  The octomap that represents additional collision data
        public Octomap.OctomapWithPoseMsg octomap;

        public PlanningSceneWorldMsg()
        {
            this.collision_objects = new CollisionObjectMsg[0];
            this.octomap = new Octomap.OctomapWithPoseMsg();
        }

        public PlanningSceneWorldMsg(CollisionObjectMsg[] collision_objects, Octomap.OctomapWithPoseMsg octomap)
        {
            this.collision_objects = collision_objects;
            this.octomap = octomap;
        }

        public static PlanningSceneWorldMsg Deserialize(MessageDeserializer deserializer) => new PlanningSceneWorldMsg(deserializer);

        private PlanningSceneWorldMsg(MessageDeserializer deserializer)
        {
            deserializer.Read(out this.collision_objects, CollisionObjectMsg.Deserialize, deserializer.ReadLength());
            this.octomap = Octomap.OctomapWithPoseMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.WriteLength(this.collision_objects);
            serializer.Write(this.collision_objects);
            serializer.Write(this.octomap);
        }

        public override string ToString()
        {
            return "PlanningSceneWorldMsg: " +
            "\ncollision_objects: " + System.String.Join(", ", collision_objects.ToList()) +
            "\noctomap: " + octomap.ToString();
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
