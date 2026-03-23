using RobotMatrix.Math;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 末端执行器绝对位姿移动命令。
    /// </summary>
    public class EndEffectorMoveCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.EndEffectorMove;

        /// <summary>目标世界位置。</summary>
        public RMVector3 TargetPosition;

        /// <summary>目标世界旋转。</summary>
        public RMQuaternion TargetRotation;

        public EndEffectorMoveCommand(RMVector3 position, RMQuaternion rotation)
        {
            TargetPosition = position;
            TargetRotation = rotation;
        }
    }

    /// <summary>
    /// 末端执行器增量移动命令（位置偏移 + 欧拉角旋转增量）。
    /// </summary>
    public class EndEffectorDeltaCommand : IControlCommand
    {
        public ControlCommandType Type => ControlCommandType.EndEffectorDelta;

        /// <summary>位置增量（世界坐标系）。</summary>
        public RMVector3 PositionDelta;

        /// <summary>旋转增量（世界坐标系，欧拉角，弧度）。</summary>
        public RMVector3 RotationDelta;

        public EndEffectorDeltaCommand(RMVector3 posDelta, RMVector3 rotDelta)
        {
            PositionDelta = posDelta;
            RotationDelta = rotDelta;
        }
    }
}
