namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 控制命令类型枚举。
    /// </summary>
    public enum ControlCommandType
    {
        /// <summary>末端执行器绝对位姿移动。</summary>
        EndEffectorMove,

        /// <summary>末端执行器增量移动。</summary>
        EndEffectorDelta,

        /// <summary>设置单个关节角度。</summary>
        JointAngle,

        /// <summary>设置所有关节角度。</summary>
        JointAnglesAll,

        /// <summary>切换控制模式。</summary>
        ModeSwitch,

        /// <summary>拖动关节保持 TCP 位置不变。</summary>
        JointDragKeepTCP
    }

    /// <summary>
    /// 控制命令接口 — 所有输入处理器产生的操作指令统一抽象。
    /// 采用命令模式，解耦输入源和控制器执行逻辑。
    /// </summary>
    public interface IControlCommand
    {
        /// <summary>命令类型标识。</summary>
        ControlCommandType Type { get; }
    }
}
