namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// IK 求解选项 — 支持锁定关节（Phase 3: 拖动关节保持 TCP 不变）。
    /// 值类型，轻量级传递。
    /// </summary>
    public struct IKSolveOptions
    {
        /// <summary>
        /// 锁定关节掩码（null 或 bool[N]）。
        /// true = 该关节被锁定，求解器不会改变其角度。
        /// </summary>
        public bool[] LockedJoints;

        /// <summary>
        /// 锁定关节的目标角度（null 或 float[N]，弧度）。
        /// 仅 LockedJoints[j] == true 的元素有效。
        /// </summary>
        public float[] LockedAngles;

        /// <summary>是否包含锁定关节信息。</summary>
        public bool HasLockedJoints => LockedJoints != null;

        /// <summary>默认选项（无锁定关节）。</summary>
        public static readonly IKSolveOptions Default = new IKSolveOptions();
    }
}
