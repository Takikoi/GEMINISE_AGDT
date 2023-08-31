using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.UrdfImporter;
using UnityEngine.Assertions;

namespace Telepatia.URControl
{
    public class OnRobotRGGripperController : MonoBehaviour
    {
        [Serializable] public struct MimicLink {public ArticulationBody link; public float multiplier;}
        [field:SerializeField] public OnRobotGripperSO GripperSO {get; private set;}
        [field:SerializeField] private ArticulationBody m_controlLink;
        [field:SerializeField] private List<MimicLink> m_mimicLinks;
        private float m_controlLinkUpperLimit, m_controlLinkLowerLimit;
        private float m_gripperFingerLength;

        [Range (0, 1100)] public float width; 
        public float target;

        void Start()
        {
            // Assert.IsNotNull<ArticulationBody>(m_controlLink, "Null Reference to Control Link");
            // Assert.IsTrue(m_mimicLinks == null, "Mimmic Links are empty.");
            Init();
        }

        void Update()
        {
            MimicControlLink();
            ArticulationDrive drive = m_controlLink.xDrive;
            drive.target = target = Mathf.Rad2Deg * (m_controlLinkUpperLimit - Mathf.Asin(width / (2f * m_gripperFingerLength)));
            m_controlLink.xDrive = drive;
        }

        private void MimicControlLink()
        {
            if (m_controlLink == null 
            || m_mimicLinks == null) return;

            for (int i = 0; i < m_mimicLinks.Capacity; i++)
            {
                if (m_mimicLinks[i].link.jointType != ArticulationJointType.RevoluteJoint) 
                {
                    Debug.LogWarning("Not a revolute joint");
                    return;
                }

                ArticulationDrive drive = m_mimicLinks[i].link.xDrive;
                drive.target = m_mimicLinks[i].multiplier * m_controlLink.xDrive.target;
                m_mimicLinks[i].link.xDrive = drive;
            }
        }

        private void Init()
        {
            m_controlLinkLowerLimit = Mathf.Deg2Rad * m_controlLink.xDrive.lowerLimit;
            m_controlLinkUpperLimit = Mathf.Deg2Rad * m_controlLink.xDrive.upperLimit;

            float range = Mathf.Abs(m_controlLinkLowerLimit) + Mathf.Abs(m_controlLinkUpperLimit);

            if (range >= Mathf.PI/2) 
            {
                Debug.LogWarning("Gripper range of motion is greater than 90 degrees !");
            }
            m_gripperFingerLength = GripperSO.MaxWidth / (2 * Mathf.Sin(range));
        }
    }
}
