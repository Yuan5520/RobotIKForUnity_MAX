namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 关节拖动保持 TCP 位置命令（Phase 3: 零空间用户交互）。
    /// 将指定关节移动到新角度，同时系统自动调整其他关节以保持 TCP 位置不变。
    /// </summary>
    public class JointDragKeepTCPCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.JointDragKeepTCP;

        /// <summary>要拖动的关节索引。</summary>
        public int JointIndex;

        /// <summary>目标角度（弧度）。</summary>
        public float NewAngleRad;

        public JointDragKeepTCPCommand(int jointIndex, float newAngleRad)
        {
            JointIndex = jointIndex;
            NewAngleRad = newAngleRad;
        }
    }
}
