using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 交互式控制柄接口（仅编辑器可用）。
    /// 提供可拖拽的位置/旋转 Handle，返回用户操作后的新值。
    /// Unity 实现：编辑器下使用 Handles API，运行时使用 NullHandle（空实现）。
    /// </summary>
    public interface IEngineHandle
    {
        /// <summary>
        /// 显示位置控制柄，返回用户拖拽后的新位置。
        /// </summary>
        RMVector3 PositionHandle(RMVector3 position, RMQuaternion rotation);

        /// <summary>
        /// 显示旋转控制柄，返回用户拖拽后的新旋转。
        /// </summary>
        RMQuaternion RotationHandle(RMQuaternion rotation, RMVector3 position);

        /// <summary>
        /// 运行时返回 false，编辑器返回 true。
        /// </summary>
        bool IsAvailable { get; }
    }
}
