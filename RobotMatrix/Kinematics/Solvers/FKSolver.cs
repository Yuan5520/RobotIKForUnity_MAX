using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 正运动学求解器。
    /// 基于 DH 矩阵链式相乘，计算各关节世界变换和末端位姿。
    /// </summary>
    public class FKSolver
    {
        /// <summary>
        /// 计算所有关节的世界变换和末端执行器位姿。
        /// </summary>
        /// <param name="model">机械臂模型（含 DH 参数）。</param>
        /// <param name="jointAngles">各关节当前角度值（弧度/位移）。</param>
        /// <returns>末端执行器的齐次变换矩阵。</returns>
        public RMMatrix4x4 Solve(RobotArmModel model, float[] jointAngles)
        {
            var T = RMMatrix4x4.Identity;

            for (int i = 0; i < model.DOF; i++)
            {
                var cfg = model.JointConfigs[i];
                var M = cfg.LinkTransform;

                RMMatrix4x4 Ti;

                if (cfg.Type == JointType.Revolute)
                {
                    // 旋转关节：T_i = M_i * Rot_z(angle)
                    // LinkTransform 先将坐标系从帧 i-1 变换到帧 i，
                    // 然后 RotZ 在帧 i 的 z 轴上旋转，确保关节 i 绕自身的物理旋转轴转动。
                    Ti = M * RMMatrix4x4.RotZ(jointAngles[i]);
                }
                else
                {
                    // 移动关节：T_i = M_i * Trans_z(displacement)
                    Ti = M * RMMatrix4x4.TransZ(jointAngles[i]);
                }

                T = T * Ti;

                model.JointStates[i].WorldTransform = T;
            }

            // 末端执行器 = 最后关节变换 * 末端偏移
            return T * model.EndEffectorOffset;
        }
    }
}
