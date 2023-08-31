using System;
using System.Linq;
using System.Collections;
using System.Threading.Tasks;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Assertions;
using MathNet.Numerics;
using MathNet.Numerics.LinearAlgebra;
using LinAlg = MathNet.Numerics.LinearAlgebra;
using Trig = MathNet.Numerics.Trig;

namespace Takikoi.Kinematics
{
    [Serializable]
    public class ElementaryTransform
    {
        public LinAlg.Matrix<double> Content {get; private set;}
        public enum Axis {Tx, Ty, Tz, Rx, Ry, Rz}
        public Axis TransformAxis {get; private set;} = Axis.Tx; 
        public bool IsFlipped {get; private set;} = false;
        public bool IsStatic {get; private set;} = true;

        public ElementaryTransform() {}
        public ElementaryTransform(LinAlg.Matrix<double> content, bool isStatic=true, bool isFlipped=false, Axis direction=Axis.Tx) 
        {
            this.Content = content;
            this.IsFlipped = isFlipped;
            this.IsStatic = isStatic;
            this.TransformAxis = direction;
        }

    //* [Method functions] 
    #region Method Functions
        public void CalculateTransform(double val=0.0)
        {
            if (IsStatic) 
            {
                Debug.LogError("The Elementary Transform is static. Can't calculate transform.");
                return;
            }
            val = val * (IsFlipped? -1.0 : 1.0);
            switch (TransformAxis)
            {
                case Axis.Tx:
                    Content.At(0, 3, val);
                    break;
                case Axis.Ty:
                    Content.At(1, 3, val);
                    break;
                case Axis.Tz:
                    Content.At(2, 3, val);
                    break;
                case Axis.Rx:
                    Content.At(1, 1, Trig.Cos(val));
                    Content.At(2, 2, Trig.Cos(val));
                    Content.At(1, 2, Trig.Sin(val) * -1.0);
                    Content.At(2, 1, Trig.Sin(val));
                    break;
                case Axis.Ry:
                    Content.At(0, 0, Trig.Cos(val));
                    Content.At(2, 2, Trig.Cos(val));
                    Content.At(2, 0, Trig.Sin(val) * -1.0);
                    Content.At(0, 2, Trig.Sin(val));
                    break;
                case Axis.Rz:
                    Content.At(0, 0, Trig.Cos(val));
                    Content.At(1, 1, Trig.Cos(val));
                    Content.At(0, 1, Trig.Sin(val) * -1.0);
                    Content.At(1, 0, Trig.Sin(val));
                    break;
            }
        }

        public LinAlg.Matrix<double> GetTranslationVectorMatrix() => Content.SubMatrix(0, 3, 3, 1);
        public LinAlg.Matrix<double> GetRotationMatrix() => Content.SubMatrix(0, 3, 0, 3);

    #endregion Method Functions

    //* [Static functions] 
    #region Static Functions

        public static ElementaryTransform Tx(double delta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> tx = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            tx.At(0, 3, delta);
            return new ElementaryTransform(tx, isStatic, isFlipped, Axis.Tx);
        }
        public static ElementaryTransform Ty(double delta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> ty = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            ty.At(1, 3, delta);
            return new ElementaryTransform(ty, isStatic, isFlipped, Axis.Ty);
        } 
        public static ElementaryTransform Tz(double delta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> tz = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            tz.At(2, 3, delta);
            return new ElementaryTransform(tz, isStatic, isFlipped, Axis.Tz);
        } 

        public static ElementaryTransform Rx(double theta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> rx = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            rx.At(1, 1, Trig.Cos(theta));
            rx.At(2, 2, Trig.Cos(theta));
            rx.At(1, 2, Trig.Sin(theta) * -1);
            rx.At(2, 1, Trig.Sin(theta));
            return new ElementaryTransform(rx, isStatic, isFlipped, Axis.Rx);
        } 
        public static ElementaryTransform Ry(double theta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> ry = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            ry.At(0, 0, Trig.Cos(theta));
            ry.At(2, 2, Trig.Cos(theta));
            ry.At(2, 0, Trig.Sin(theta) * -1);
            ry.At(0, 2, Trig.Sin(theta));
            return new ElementaryTransform(ry, isStatic, isFlipped, Axis.Ry);
        }
        public static ElementaryTransform Rz(double theta=0.0, bool isStatic=true, bool isFlipped=false)
        {
            LinAlg.Matrix<double> rz = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            rz.At(0, 0, Trig.Cos(theta));
            rz.At(1, 1, Trig.Cos(theta));
            rz.At(0, 1, Trig.Sin(theta) * -1);
            rz.At(1, 0, Trig.Sin(theta));
            return new ElementaryTransform(rz, isStatic, isFlipped, Axis.Rz);
        }

        //* Raw matrix
        public static LinAlg.Matrix<double> tx(double delta=0.0)
        {
            LinAlg.Matrix<double> tx = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            tx.At(0, 3, delta);
            return tx;
        }
        public static LinAlg.Matrix<double> ty(double delta=0.0)
        {
            LinAlg.Matrix<double> ty = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            ty.At(1, 3, delta);
            return ty;
        } 
        public static LinAlg.Matrix<double> tz(double delta=0.0)
        {
            LinAlg.Matrix<double> tz = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            tz.At(2, 3, delta);
            return tz;
        } 

        public static LinAlg.Matrix<double> rx(double theta=0.0)
        {
            LinAlg.Matrix<double> rx = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            rx.At(1, 1, Trig.Cos(theta));
            rx.At(2, 2, Trig.Cos(theta));
            rx.At(1, 2, Trig.Sin(theta) * -1);
            rx.At(2, 1, Trig.Sin(theta));
            return rx;
        } 
        public static LinAlg.Matrix<double> ry(double theta=0.0)
        {
            LinAlg.Matrix<double> ry = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            ry.At(0, 0, Trig.Cos(theta));
            ry.At(2, 2, Trig.Cos(theta));
            ry.At(2, 0, Trig.Sin(theta) * -1);
            ry.At(0, 2, Trig.Sin(theta));
            return ry;
        }
        public static LinAlg.Matrix<double> rz(double theta=0.0)
        {
            LinAlg.Matrix<double> rz = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            rz.At(0, 0, Trig.Cos(theta));
            rz.At(1, 1, Trig.Cos(theta));
            rz.At(0, 1, Trig.Sin(theta) * -1);
            rz.At(1, 0, Trig.Sin(theta));
            return rz;
        }

    #endregion Static Functions
    }

    [Serializable]
    public class ElementaryTransformSequence
    {
        public ElementaryTransform[] Sequence {get; private set;}
        public int DOF {get; private set;} = 0;

        public ElementaryTransformSequence() {}
        public ElementaryTransformSequence(ElementaryTransform[] sequence)
        {
            if (!sequence[0].IsStatic)
            {
                Debug.LogError("Robot base is not static.");
                return;
            }

            this.Sequence = sequence;
            this.DOF = 0;
            List<ElementaryTransform> variableETs = new List<ElementaryTransform>();

            foreach (ElementaryTransform et in sequence)
            {
                if (et.IsStatic) {continue;}
                DOF++;
            }

            Debug.Log($"DOF: {DOF}");
        }

        public LinAlg.Matrix<double> CalculateFK(double[] jointPositions, int startLinkIndex=0, int endLinkIndex=-1)
        {
            /// <summary>
            /// This method takes joint positions to calculate transform matrix from startlink -> endlink.
            /// start = 0: the base link
            /// end < 0: the end-effector link
            /// 
            /// If the FK is from base to eef, than it's call Pose
            /// </summary>
            
            if (startLinkIndex < 0 || (endLinkIndex > 0 && startLinkIndex > endLinkIndex)) 
            {
                Debug.LogAssertion("Error in start and end link indexes when calculating FK.");
                return LinAlg.CreateMatrix.DenseIdentity<double>(4);
            }
            if (jointPositions.Length < DOF) 
            {
                Debug.LogWarning("Joint positions is less than DOF.");
                Array.Resize(ref jointPositions, jointPositions.Length + (DOF - jointPositions.Length));
            }
            else if (jointPositions.Length > DOF) 
            {
                Debug.LogWarning("Joint positions is more than DOF.");
                Array.Resize(ref jointPositions, jointPositions.Length + (DOF - jointPositions.Length));
            }

            // if simply calculating from base to end-effector (Pose), this is faster
            if (startLinkIndex == 0 && endLinkIndex < 0)
            {
                int j = 0;
                LinAlg.Matrix<double> base2eef = LinAlg.CreateMatrix.DenseIdentity<double>(4);

                for (int i = 0; i < Sequence.Length; i++)
                {   
                    if (!Sequence[i].IsStatic) 
                    {
                        Sequence[i].CalculateTransform(jointPositions[j++]);
                    }
                    base2eef = base2eef.Multiply(Sequence[i].Content);
                }
                return base2eef;
            }
            else // (startLinkIndex != 0 && endLinkIndex >= 0)
            {
                int j1 = 0, j2 = 0;
                LinAlg.Matrix<double> start = LinAlg.CreateMatrix.DenseIdentity<double>(4);
                LinAlg.Matrix<double> end = LinAlg.CreateMatrix.DenseIdentity<double>(4);

                for (int i = 0; i < Sequence.Length; i++)
                {   
                    if (!Sequence[i].IsStatic) 
                    {
                        Sequence[i].CalculateTransform(jointPositions[j1]);
                    }
                    start = start.Multiply(Sequence[i].Content);
                    if (j1 == startLinkIndex) {break;}
                    j1++;
                }

                for (int i = 0; i < Sequence.Length; i++)
                {   
                    if (!Sequence[i].IsStatic) 
                    {
                        Sequence[i].CalculateTransform(jointPositions[j2]);
                    }
                    end = end.Multiply(Sequence[i].Content);
                    if (j2 == endLinkIndex) {break;}
                    j2++;
                }
                return start.Inverse().Multiply(end);
            }

        }

        public LinAlg.Matrix<double> CalculateFastJacobian(double[] jointPositions)
        {
            /// <summary>
            /// This method takes joint positions of each angle to calculate fast-jacobian of end-effector wrt base.
            /// See Peter Corke's Manipulator Differential Kinematics Part 1 
            /// </summary>
            
            LinAlg.Matrix<double> jacobian = LinAlg.CreateMatrix.Dense<double>(6, DOF);
            LinAlg.Matrix<double> base2eef = CalculateFK(jointPositions);
            LinAlg.Matrix<double> u = LinAlg.CreateMatrix.DenseIdentity<double>(4);

            int j = 0, k = 0;
            for (int i = 0; i < Sequence.Length; i++)
            {
                if (Sequence[i].IsStatic)
                {
                    u = u.Multiply(Sequence[i].Content);
                }
                else
                {
                    Sequence[i].CalculateTransform(jointPositions[j++]);
                    u = u.Multiply(Sequence[i].Content);
                    LinAlg.Matrix<double> tu = u.Inverse().Multiply(base2eef);

                    LinAlg.Matrix<double> n = u.SubMatrix(0, 3, 0, 1);
                    LinAlg.Matrix<double> o = u.SubMatrix(0, 3, 1, 1);
                    LinAlg.Matrix<double> a = u.SubMatrix(0, 3, 2, 1);
                    double x = tu.At(0, 3);
                    double y = tu.At(1, 3);
                    double z = tu.At(2, 3);

                    double sign = Sequence[i].IsFlipped? -1.0 : 1.0;
                    switch (Sequence[i].TransformAxis)
                    {
                        case ElementaryTransform.Axis.Tx:
                            jacobian.SetSubMatrix(0, k, n);
                            jacobian.SetSubMatrix(3, k, LinAlg.CreateMatrix.Dense<double>(3, 1));
                            break;
                        case ElementaryTransform.Axis.Ty:
                            jacobian.SetSubMatrix(0, k, o);
                            jacobian.SetSubMatrix(3, k, LinAlg.CreateMatrix.Dense<double>(3, 1));
                            break;
                        case ElementaryTransform.Axis.Tz:
                            jacobian.SetSubMatrix(0, k, a);
                            jacobian.SetSubMatrix(3, k, LinAlg.CreateMatrix.Dense<double>(3, 1));
                            break;
                        case ElementaryTransform.Axis.Rx:
                            jacobian.SetSubMatrix(0, k, a.Multiply(y).Subtract(o.Multiply(z)).Multiply(sign));
                            jacobian.SetSubMatrix(3, k, n.Multiply(sign));
                            break;
                        case ElementaryTransform.Axis.Ry:
                            jacobian.SetSubMatrix(0, k, n.Multiply(z).Subtract(a.Multiply(x)).Multiply(sign));
                            jacobian.SetSubMatrix(3, k, o.Multiply(sign));
                            break;
                        case ElementaryTransform.Axis.Rz:
                            jacobian.SetSubMatrix(0, k, o.Multiply(x).Subtract(n.Multiply(y)).Multiply(sign));
                            jacobian.SetSubMatrix(3, k, a.Multiply(sign));
                            break;
                    }
                    k++;
                }
            }
            return jacobian;
        }

    }

    [Serializable]
    public abstract class IKSolverBase
    {
        public struct IKResult {
            public double[] JointPositions;
            public bool IsSolved;
            public double Error;
            public int TotalIter;
            public int ToTalSearch;
            public IKResult(double[] jointPositions, bool isSolved, double error, int totalIter, int totalSearch) 
            {
                this.JointPositions = new double[jointPositions.Length];
                jointPositions.CopyTo(this.JointPositions, 0);
                this.IsSolved = isSolved;
                this.Error = error;
                this.TotalIter = totalIter;
                this.ToTalSearch = totalSearch;
            }
            public void LogResult()
            {
                Debug.Log($"Success: {this.IsSolved}, Err: {this.Error}, Iter: {this.TotalIter}, Search: {this.ToTalSearch}");
            }
        }
        public string Name = "IKSolver";
        public int IterLimit {get; protected set;} = 30;
        public int SearchLimit {get; protected set;} = 100;
        public const double Tolerance = 1e-6;

        // Weight for each DOF
        public LinAlg.Matrix<double> Weight {get; protected set;} = LinAlg.CreateMatrix.DenseIdentity<double>(6);

        public IKSolverBase(int iterLimit=30, int searchLimit=100) 
        {
            this.IterLimit = iterLimit;
            this.SearchLimit = searchLimit;
        }

        public abstract double Step(ElementaryTransformSequence ets, LinAlg.Matrix<double> desiredPose, ref double[] startJointPosition);

        public IKResult Solve(ElementaryTransformSequence ets, LinAlg.Matrix<double> desiredPose, double[][] startJointPositionsList)
        {
            /// <summary>
            /// This method chooses a position in startingJointPositions (randomly generated) and 
            /// iterate it Iterlimit times until the pose error is under the tolerance. 
            /// If not, it will choose the next position to re-iterate again
            /// 
            /// </summary>

            int iter = 0, iterTotal = 0;
            double[] pos = new double[ets.DOF];
            double error = 0;
            for (int search = 0; search < SearchLimit; search++)
            {
                startJointPositionsList[search].CopyTo(pos, 0);
                while (iter <= IterLimit)
                {
                    iter++;
                    error = Step(ets, desiredPose, ref pos);
                    if (error < Tolerance) 
                    {
                        return new IKResult(pos, true, error, iterTotal + iter, ++search);
                    }
                }
                iterTotal += iter;
                iter = 0;
            }
            Debug.Log("Target Pose out-of-bound or failed.");
            return new IKResult(pos, false, error, iterTotal, SearchLimit);
        }

        public IKResult Solve(ElementaryTransformSequence ets, Transform targetPose, double[][] startJointPositionsList)
        {
            /// <summary>
            /// This method chooses a position in startingJointPositions (randomly generated) and 
            /// iterate it Iterlimit times until the pose error is under the tolerance. 
            /// If not, it will choose the next position to re-iterate again
            /// 
            /// </summary>
            
            // Position
            LinAlg.Matrix<double> desiredPos = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            desiredPos.At(0, 3, targetPose.localPosition.z * -1);
            desiredPos.At(1, 3, targetPose.localPosition.x);
            desiredPos.At(2, 3, targetPose.localPosition.y);

            // Orientation
            LinAlg.Matrix<double> rx = ElementaryTransform.rx(targetPose.eulerAngles.z * Mathf.Deg2Rad);
            LinAlg.Matrix<double> ry = ElementaryTransform.ry(targetPose.eulerAngles.x * -Mathf.Deg2Rad);
            LinAlg.Matrix<double> rz = ElementaryTransform.rz(targetPose.eulerAngles.y * -Mathf.Deg2Rad);

            LinAlg.Matrix<double> desiredPose = LinAlg.CreateMatrix.DenseIdentity<double>(4);
            desiredPose = desiredPos.Add(rz.Multiply(ry).Multiply(rx));

            int iter = 0, iterTotal = 0;
            double[] pos = new double[ets.DOF];
            double error = 0;
            for (int search = 0; search < SearchLimit; search++)
            {
                startJointPositionsList[search].CopyTo(pos, 0);
                while (iter <= IterLimit)
                {
                    iter++;
                    error = Step(ets, desiredPose, ref pos);
                    if (error < Tolerance) 
                    {
                        return new IKResult(pos, true, error, iterTotal + iter, ++search);
                    }
                }
                iterTotal += iter;
                iter = 0;
            }
            Debug.Log("Target Pose out-of-bound or failed.");
            return new IKResult(pos, false, error, iterTotal, SearchLimit);
        }   

        public double CalculateCostError(LinAlg.Matrix<double> currentPose, LinAlg.Matrix<double> desiredPose, out LinAlg.Vector<double> poseError)
        {
            poseError = CalculatePoseError(currentPose, desiredPose);
            LinAlg.Matrix<double> poseErrMatrix = LinAlg.CreateMatrix.DenseOfColumnVectors<double>(poseError);
            return poseErrMatrix.Multiply(0.5).Transpose().Multiply(Weight).Multiply(poseErrMatrix).At(0, 0);
        }

        public void SetWeightMatrix(LinAlg.Matrix<double> weight)
        {
            if (weight.ColumnCount != 6 || weight.RowCount != 6) 
            {
                Debug.LogError("Weight matrix not 6x6. Cartesian only have 6 parameters (x,y,z.rx,ry,rz).");
                return;
            }
            this.Weight = weight;
        }

        // public double[][] GenerateRandomJointPositions(int size, int dof)
        // {
        //     double[][] randPositions = new double[size][dof];
        // }

    #region Static Functions
        public static LinAlg.Vector<double> CalculatePoseError(LinAlg.Matrix<double> currentPose, LinAlg.Matrix<double> desiredPose)
        {
            /// <summary>
            /// Pose is the base2eef transform matrix
            /// This method calculate the error vector matrix from 2 pose
            /// return (translation error, rotation error)
            /// </summary>
            
            LinAlg.Vector<double> error = LinAlg.CreateVector.Dense<double>(6);

            // Translation error
            error.SetSubVector(0, 3, desiredPose.Column(3, 0, 3).Subtract(currentPose.Column(3, 0, 3)));

            // Rotation error
            LinAlg.Matrix<double> rotErrMatrix = desiredPose.SubMatrix(0, 3, 0, 3).Multiply(currentPose.SubMatrix(0, 3, 0, 3).Transpose());

            LinAlg.Vector<double> li = LinAlg.CreateVector.Dense<double>(
                new double[] {
                    rotErrMatrix.At(2, 1) - rotErrMatrix.At(1, 2),
                    rotErrMatrix.At(0, 2) - rotErrMatrix.At(2, 0),
                    rotErrMatrix.At(1, 0) - rotErrMatrix.At(0, 1)
                }
            );

            LinAlg.Vector<double> rotErrVector;
            double ln = li.L2Norm();
            if (ln < 1e-6) // check if li is a zero vector or close to 0
            {
                rotErrVector = (rotErrMatrix.Trace() > 0.0)? 
                LinAlg.CreateVector.Dense<double>(3) : 
                rotErrMatrix.Diagonal().Add(1.0).Multiply(Constants.Pi * 0.5);
                // Debug.Log("from 1");
            }
            else
            {
                rotErrVector = li.Multiply(Math.Atan2(ln, rotErrMatrix.Trace() - 1.0) / ln);
                // Debug.Log($"Trace: {rotErrMatrix.Trace()}");
                // Debug.Log($"li: {li}");
                // Debug.Log($"ln: {ln}");
                // Debug.Log($"Atam: {Math.Atan2(ln , (rotErrMatrix.Trace() - 1.0))}");
            }
            error.SetSubVector(3, 3, rotErrVector);
            return error;
        }
    #endregion Static Functions
    }

    [Serializable]
    public class LevenbergMarquardtIKSolver
    {
        public class Chan : IKSolverBase
        {
            public new string Name = "LM (Chan)";
            public double Lamda {get; private set;}

            public Chan(double lamda=1.0, int iterLimit=30, int searchLimit=100)
            : base(iterLimit, searchLimit)
            {
                this.Lamda = lamda;
            }

            public override double Step(ElementaryTransformSequence ets, Matrix<double> desiredPose, ref double[] currentJointPos)
            {
                /// <summary>
                /// 
                /// </summary>

                LinAlg.Vector<double> poseErr;
                LinAlg.Matrix<double> currentPose = ets.CalculateFK(currentJointPos);
                double costErr = CalculateCostError(currentPose, desiredPose, out poseErr);

                LinAlg.Matrix<double> Wn = LinAlg.CreateMatrix.DenseIdentity<double>(ets.DOF);
                Wn = Wn.Multiply(this.Lamda * costErr);

                LinAlg.Matrix<double> jacobian = ets.CalculateFastJacobian(currentJointPos);
                LinAlg.Vector<double> g = jacobian.Transpose().Multiply(this.Weight).Multiply(poseErr);
                
                double[] delta = jacobian.Transpose().Multiply(this.Weight).Multiply(jacobian).Add(Wn).Inverse().Multiply(g).ToArray(); 
                
                for (int i = 0; i < currentJointPos.Length; i++)
                {
                    currentJointPos[i] = currentJointPos[i] + delta[i];
                }

                return costErr;
            }
        }
    }
}