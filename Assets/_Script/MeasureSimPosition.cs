using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;
using RosMessageTypes.Control;
using RosMessageTypes.UnityMoveitPlanningInterface;

/// <summary>
///
/// </summary>
public class MeasureSimPosition : MonoBehaviour
{
    public ArticulationBody[] ArticuJoints = new ArticulationBody[6];
    public Transform[] Tags;
    public float[] MeasuredPosition = new float[6];
    public float[] MeasuredVelocity = new float[6];
    private float[] LastMeasuredPosition = new float[6];
    public Transform EETransform, RootTransform;
    private Transform lastEETransform, EERelativeTransform;
    ROSConnection ros;
    public string topic_joint = "sim_joint";
    public string topic_ee = "sim_pose";

    Vector3 lastPos = new();
    Quaternion currentRot = new();

    public float publishTime = 0.02f;
    private float timeElapsed = 0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<JointTrajectoryControllerStateMsg>(topic_joint);
        ros.RegisterPublisher<EEPoseMsg>(topic_ee);
        ROSConnection.GetOrCreateInstance().Subscribe<JointTrajectoryControllerStateMsg>("/scaled_pos_joint_traj_controller/state", Mimick);
        // ROSConnection.GetOrCreateInstance().Subscribe<JointTrajectoryControllerStateMsg>("/eff_joint_traj_controller/state", Mimick);
    }

    void FixedUpdate()
    {
        // EERelativeTransform.in
        timeElapsed += Time.fixedDeltaTime;

        for (int i = 0; i < ArticuJoints.Length; i++)
        {
            MeasuredPosition[i] = Vector3.SignedAngle(Tags[i].forward, Tags[i+1].forward, Tags[i+1].transform.right) * Mathf.Deg2Rad;
            MeasuredVelocity[i] = (MeasuredPosition[i] - LastMeasuredPosition[i]) / Time.fixedDeltaTime;
        }
        MeasuredPosition.CopyTo(LastMeasuredPosition, 0);

        if (timeElapsed >= publishTime)
        {
            JointTrajectoryControllerStateMsg joint_msg = new();
            joint_msg.actual.positions = new double[6];
            joint_msg.actual.velocities = new double[6];
            MeasuredPosition.CopyTo(joint_msg.actual.positions, 0);
            MeasuredVelocity.CopyTo(joint_msg.actual.velocities, 0);
            
            Vector3 baseToEE = EETransform.position - RootTransform.position;

            EEPoseMsg pose_msg = new();
            pose_msg.pose_pos.position.x = baseToEE.z;
            pose_msg.pose_pos.position.y = baseToEE.x * -1f;
            pose_msg.pose_pos.position.z = baseToEE.y;

            pose_msg.pose_vel.position.x = (baseToEE.z - lastPos.z) / Time.fixedDeltaTime;
            pose_msg.pose_vel.position.y = ((baseToEE.x * -1f) - (lastPos.z * -1f)) / Time.fixedDeltaTime;
            pose_msg.pose_vel.position.z = (baseToEE.y - lastPos.z / Time.fixedDeltaTime);

            lastPos.x = baseToEE.z;
            lastPos.y = baseToEE.x * -1f;
            lastPos.z = baseToEE.y;


            // pose_msg.pose_pos.orientation.x = EETransform.rotation.w;
            // pose_msg.pose_pos.orientation.y = EETransform.rotation.y * -1f;
            // pose_msg.pose_pos.orientation.z = EETransform.rotation.x * -1f;
            // pose_msg.pose_pos.orientation.w = EETransform.rotation.z;

            // pose_msg.pose_pos.orientation.x = -EETransform.rotation.w;
            // pose_msg.pose_pos.orientation.y = EETransform.rotation.y;
            // pose_msg.pose_pos.orientation.z = EETransform.rotation.x;
            // pose_msg.pose_pos.orientation.w = -EETransform.rotation.z;

            // pose_msg.pose_vel.orientation.x = -EETransform.rotation.w - -currentRot.w;
            // pose_msg.pose_vel.orientation.x = EETransform.rotation.y - currentRot.y;
            // pose_msg.pose_vel.orientation.y = EETransform.rotation.x - currentRot.x;
            // pose_msg.pose_vel.orientation.z = -EETransform.rotation.z - -currentRot.z;

            // currentRot.x = -EETransform.rotation.w;
            // currentRot.y = EETransform.rotation.y;
            // currentRot.z = EETransform.rotation.x;
            // currentRot.w = -EETransform.rotation.z;

            ros.Publish(topic_joint, joint_msg);
            ros.Publish(topic_ee, pose_msg);

            timeElapsed = 0f;
        }
    }

    public void Mimick(JointTrajectoryControllerStateMsg callbackMsg)
    {
        for (int i = 0; i < ArticuJoints.Length; i++)
        {
            ArticulationDrive drive =  ArticuJoints[i].xDrive;
            drive.target = (float)callbackMsg.desired.positions[i] * Mathf.Rad2Deg;
            ArticuJoints[i].xDrive = drive;
        }
    }
}