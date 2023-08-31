using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Takikoi.Kinematics;
using MathNet.Numerics;
using Trig = MathNet.Numerics.Trig;
using LinAlg = MathNet.Numerics.LinearAlgebra;
using LM = Takikoi.Kinematics.LevenbergMarquardtIKSolver;

public class RobotIKController : MonoBehaviour
{
    public ArticulationBody[] Joints;
    public LM.Chan ChanSolver = new LM.Chan(lamda: 0.1, searchLimit:6);
    public Transform TargetPose;
    public ElementaryTransformSequence ets;

    public ElementaryTransform E1 = ElementaryTransform.Tz(0.1807);
    public ElementaryTransform EJ1 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E2 = ElementaryTransform.Rx(Constants.Pi * 0.5);
    public ElementaryTransform EJ2 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E3 = ElementaryTransform.Tx(-0.6127);
    public ElementaryTransform EJ3 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E4 = ElementaryTransform.Tx(-0.57155);
    public ElementaryTransform EJ4 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E5 = ElementaryTransform.Tz(0.17415);
    public ElementaryTransform E6 = ElementaryTransform.Rx(Constants.Pi * 0.5);
    public ElementaryTransform EJ6 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E7 = ElementaryTransform.Tz(0.11985);
    public ElementaryTransform E8 = ElementaryTransform.Rx(Constants.Pi * -0.5);
    public ElementaryTransform EJ8 = ElementaryTransform.Rz(isStatic:false);
    public ElementaryTransform E9 = ElementaryTransform.Tz(0.11655);

    double[][] q0 = new [] {
        new [] {0.0,  0.0, 0.0, 0.0,  0.0,  0.0},
        new [] {-2.74,  0.55, -1.18, -0.76,  0.36,  0.66},
        new [] { 1.13, -0.79, -1.94, -1.32, -1.25,  0.44},
        new [] {-0.58,  0.83, -1.78, -2.14,  1.25,  1.94},
        new [] { 2.06,  1.14, -2.36, -1.92, -2.27,  0.43},
        new [] {-0.54, -1.13,  0.72, -0.45, -0.11,  1.55}
    };
    
    void Start()
    {
        List<ElementaryTransform> sq = new List<ElementaryTransform>();
        sq.Add(E1);
        sq.Add(EJ1);
        sq.Add(E2);
        sq.Add(EJ2);
        sq.Add(E3);
        sq.Add(EJ3);
        sq.Add(E4);
        sq.Add(EJ4);
        sq.Add(E5);
        sq.Add(E6);
        sq.Add(EJ6);
        sq.Add(E7);
        sq.Add(E8);
        sq.Add(E9);
        sq.Add(EJ8);

        ets = new ElementaryTransformSequence(sq.ToArray());
    }

    void Update()
    {
        // LinAlg.Matrix<double> desiredPose = LinAlg.CreateMatrix.DenseIdentity<double>(4);
        // desiredPose.At(0, 3, TargetPose.localPosition.z * -1);
        // desiredPose.At(1, 3, TargetPose.localPosition.x);
        // desiredPose.At(2, 3, TargetPose.localPosition.y);
        
        
        IKSolverBase.IKResult result = ChanSolver.Solve(ets, TargetPose, q0);
        result.LogResult();
        for (int i = 0; i < Joints.Length; i++)
        {
            ArticulationDrive drive = Joints[i].xDrive; 
            drive.target = (float)result.JointPositions[i] * Mathf.Rad2Deg;
            Joints[i].xDrive = drive;
        }
    }
}
