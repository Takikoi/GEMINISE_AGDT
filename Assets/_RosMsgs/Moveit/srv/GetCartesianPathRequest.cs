//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    [Serializable]
    public class GetCartesianPathRequest : Message
    {
        public const string k_RosMessageName = "moveit_msgs/GetCartesianPath";
        public override string RosMessageName => k_RosMessageName;

        //  Define the frame for the specified waypoints
        public HeaderMsg header;
        //  The start at which to start the Cartesian path
        public RobotStateMsg start_state;
        //  Mandatory name of group to compute the path for
        public string group_name;
        //  Optional name of IK link for which waypoints are specified.
        //  If not specified, the tip of the group (which is assumed to be a chain)
        //  is assumed to be the link
        public string link_name;
        //  A sequence of waypoints to be followed by the specified link,
        //  while moving the specified group, such that the group moves only
        //  in a straight line between waypoints
        public Geometry.PoseMsg[] waypoints;
        //  The maximum distance (in Cartesian space) between consecutive points
        //  in the returned path. This must always be specified and > 0
        public double max_step;
        //  If jump_threshold is set > 0, it acts as a scaling factor that is used to
        //  filter out large relative joint-space jumps in the generated Cartesian path.
        //  To this end, the average joint-space distance between consecutive waypoints
        //  is computed. If any joint-space distance is larger than this average distance
        //  by a factor of jump_threshold_factor, this step is considered a jump
        //  and the returned path is truncated before the step.
        public double jump_threshold;
        //  If prismatic_jump_threshold or revolute_jump_threshold are set > 0, then for
        //  all active prismatic or revolute joints, the joint-space difference between
        //  consecutive waypoints is compared to the respective absolute threshold.
        //  If any threshold is exceeded, this step is considered a jump and the returned path
        //  is truncated before the step.
        public double prismatic_jump_threshold;
        public double revolute_jump_threshold;
        //  Set to true if collisions should be avoided when possible
        public bool avoid_collisions;
        //  Specify additional constraints to be met by the Cartesian path
        public ConstraintsMsg path_constraints;
        //  Maximum cartesian speed for the given link.
        //  If max_cartesian_speed <= 0 the trajectory is not modified.
        public string cartesian_speed_limited_link;
        public double max_cartesian_speed;
        //  m/s

        public GetCartesianPathRequest()
        {
            this.header = new HeaderMsg();
            this.start_state = new RobotStateMsg();
            this.group_name = "";
            this.link_name = "";
            this.waypoints = new Geometry.PoseMsg[0];
            this.max_step = 0.0;
            this.jump_threshold = 0.0;
            this.prismatic_jump_threshold = 0.0;
            this.revolute_jump_threshold = 0.0;
            this.avoid_collisions = false;
            this.path_constraints = new ConstraintsMsg();
            this.cartesian_speed_limited_link = "";
            this.max_cartesian_speed = 0.0;
        }

        public GetCartesianPathRequest(HeaderMsg header, RobotStateMsg start_state, string group_name, string link_name, Geometry.PoseMsg[] waypoints, double max_step, double jump_threshold, double prismatic_jump_threshold, double revolute_jump_threshold, bool avoid_collisions, ConstraintsMsg path_constraints, string cartesian_speed_limited_link, double max_cartesian_speed)
        {
            this.header = header;
            this.start_state = start_state;
            this.group_name = group_name;
            this.link_name = link_name;
            this.waypoints = waypoints;
            this.max_step = max_step;
            this.jump_threshold = jump_threshold;
            this.prismatic_jump_threshold = prismatic_jump_threshold;
            this.revolute_jump_threshold = revolute_jump_threshold;
            this.avoid_collisions = avoid_collisions;
            this.path_constraints = path_constraints;
            this.cartesian_speed_limited_link = cartesian_speed_limited_link;
            this.max_cartesian_speed = max_cartesian_speed;
        }

        public static GetCartesianPathRequest Deserialize(MessageDeserializer deserializer) => new GetCartesianPathRequest(deserializer);

        private GetCartesianPathRequest(MessageDeserializer deserializer)
        {
            this.header = HeaderMsg.Deserialize(deserializer);
            this.start_state = RobotStateMsg.Deserialize(deserializer);
            deserializer.Read(out this.group_name);
            deserializer.Read(out this.link_name);
            deserializer.Read(out this.waypoints, Geometry.PoseMsg.Deserialize, deserializer.ReadLength());
            deserializer.Read(out this.max_step);
            deserializer.Read(out this.jump_threshold);
            deserializer.Read(out this.prismatic_jump_threshold);
            deserializer.Read(out this.revolute_jump_threshold);
            deserializer.Read(out this.avoid_collisions);
            this.path_constraints = ConstraintsMsg.Deserialize(deserializer);
            deserializer.Read(out this.cartesian_speed_limited_link);
            deserializer.Read(out this.max_cartesian_speed);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.header);
            serializer.Write(this.start_state);
            serializer.Write(this.group_name);
            serializer.Write(this.link_name);
            serializer.WriteLength(this.waypoints);
            serializer.Write(this.waypoints);
            serializer.Write(this.max_step);
            serializer.Write(this.jump_threshold);
            serializer.Write(this.prismatic_jump_threshold);
            serializer.Write(this.revolute_jump_threshold);
            serializer.Write(this.avoid_collisions);
            serializer.Write(this.path_constraints);
            serializer.Write(this.cartesian_speed_limited_link);
            serializer.Write(this.max_cartesian_speed);
        }

        public override string ToString()
        {
            return "GetCartesianPathRequest: " +
            "\nheader: " + header.ToString() +
            "\nstart_state: " + start_state.ToString() +
            "\ngroup_name: " + group_name.ToString() +
            "\nlink_name: " + link_name.ToString() +
            "\nwaypoints: " + System.String.Join(", ", waypoints.ToList()) +
            "\nmax_step: " + max_step.ToString() +
            "\njump_threshold: " + jump_threshold.ToString() +
            "\nprismatic_jump_threshold: " + prismatic_jump_threshold.ToString() +
            "\nrevolute_jump_threshold: " + revolute_jump_threshold.ToString() +
            "\navoid_collisions: " + avoid_collisions.ToString() +
            "\npath_constraints: " + path_constraints.ToString() +
            "\ncartesian_speed_limited_link: " + cartesian_speed_limited_link.ToString() +
            "\nmax_cartesian_speed: " + max_cartesian_speed.ToString();
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
