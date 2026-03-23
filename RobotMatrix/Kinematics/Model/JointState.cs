using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 运行时关节状态。
    /// </summary>
    public class JointState
    {
        /// <summary>当前关节值（旋转关节=角度弧度，移动关节=位移）。</summary>
        public float CurrentAngle;

        /// <summary>该关节的世界变换矩阵（FK 计算结果）。</summary>
        public RMMatrix4x4 WorldTransform = RMMatrix4x4.Identity;
    }
}
