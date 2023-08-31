using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Telepatia.URControl
{
    [CreateAssetMenu(fileName = "OnRobotGripperSO", menuName = "ScriptableObjects/OnRobot", order = 1)]
    public class OnRobotGripperSO : ScriptableObject
    {
        [field:SerializeField] public OnRobotGripperModel Model {get; private set;}
        [field:SerializeField] public float MaxWidth {get; private set;}
        [field:SerializeField] public float MaxForce {get; private set;}
        
    }
    public enum OnRobotGripperModel {RG2, RG6}
}