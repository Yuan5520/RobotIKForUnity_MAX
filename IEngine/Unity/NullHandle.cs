using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// 空实现的控制柄，运行时回退使用。
    /// IsAvailable 始终返回 false，Handle 方法原样返回输入值。
    /// </summary>
    public class NullHandle : IEngineHandle
    {
        public RMVector3 PositionHandle(RMVector3 position, RMQuaternion rotation)
        {
            return position;
        }

        public RMQuaternion RotationHandle(RMQuaternion rotation, RMVector3 position)
        {
            return rotation;
        }

        public bool IsAvailable => false;
    }
}
