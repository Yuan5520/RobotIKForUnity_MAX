using RobotMatrix.Controller;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 控制模式切换命令。
    /// </summary>
    public class ModeSwitchCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.ModeSwitch;

        /// <summary>目标控制模式。</summary>
        public ControlMode TargetMode;

        public ModeSwitchCommand(ControlMode targetMode)
        {
            TargetMode = targetMode;
        }
    }
}
