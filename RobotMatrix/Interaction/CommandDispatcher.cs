using RobotMatrix.Controller;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 命令分发器 — 将 IControlCommand 应用到 RobotArmController。
    /// 集中处理命令路由，避免输入处理器直接耦合控制器内部 API。
    /// </summary>
    public class CommandDispatcher
    {
        private readonly RobotArmController _controller;

        public CommandDispatcher(RobotArmController controller)
        {
            _controller = controller;
        }

        /// <summary>
        /// 分发并执行命令。
        /// </summary>
        /// <returns>命令是否被成功处理。</returns>
        public bool Dispatch(IControlCommand command)
        {
            if (command == null || !_controller.IsInitialized)
                return false;

            switch (command.Type)
            {
                case ControlCommandType.EndEffectorMove:
                    var moveCmd = (EndEffectorMoveCommand)command;
                    _controller.MoveEndEffectorTo(moveCmd.TargetPosition, moveCmd.TargetRotation);
                    return true;

                case ControlCommandType.EndEffectorDelta:
                    var deltaCmd = (EndEffectorDeltaCommand)command;
                    _controller.MoveEndEffectorDelta(deltaCmd.PositionDelta, deltaCmd.RotationDelta);
                    return true;

                case ControlCommandType.JointAngle:
                    var angleCmd = (JointAngleCommand)command;
                    _controller.SetJointAngle(angleCmd.JointIndex, angleCmd.Angle);
                    return true;

                case ControlCommandType.JointAnglesAll:
                    var allCmd = (JointAnglesAllCommand)command;
                    _controller.SetJointAngles(allCmd.Angles);
                    return true;

                case ControlCommandType.ModeSwitch:
                    var modeCmd = (ModeSwitchCommand)command;
                    _controller.SetControlMode(modeCmd.TargetMode);
                    return true;

                case ControlCommandType.JointDragKeepTCP:
                    var dragCmd = (JointDragKeepTCPCommand)command;
                    _controller.AdjustJointKeepTCP(dragCmd.JointIndex, dragCmd.NewAngleRad);
                    return true;

                default:
                    return false;
            }
        }
    }
}
