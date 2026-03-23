namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 设置单个关节角度命令。
    /// </summary>
    public class JointAngleCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.JointAngle;

        /// <summary>目标关节索引。</summary>
        public int JointIndex;

        /// <summary>目标角度值（弧度）。</summary>
        public float Angle;

        public JointAngleCommand(int jointIndex, float angle)
        {
            JointIndex = jointIndex;
            Angle = angle;
        }
    }

    /// <summary>
    /// 设置所有关节角度命令。
    /// </summary>
    public class JointAnglesAllCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.JointAnglesAll;

        /// <summary>所有关节角度（弧度）。</summary>
        public float[] Angles;

        public JointAnglesAllCommand(float[] angles)
        {
            Angles = angles;
        }
    }
}
