using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Controller;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 编辑器手柄输入处理器。
    /// 使用 IEngineHandle 在 Scene 视图中显示位置/旋转手柄，
    /// 检测手柄拖拽产生的位姿变化并生成对应的控制命令。
    /// 仅在编辑器环境下有效（运行时 IEngineHandle.IsAvailable = false）。
    /// </summary>
    public class HandleInputHandler : IInputHandler
    {
        private readonly IEngineHandle _handle;
        private readonly RobotArmController _controller;

        private RMVector3 _lastHandlePosition;
        private RMQuaternion _lastHandleRotation;
        private bool _initialized;

        public bool Enabled { get; set; } = true;

        public HandleInputHandler(IEngineHandle handle, RobotArmController controller)
        {
            _handle = handle;
            _controller = controller;
        }

        public IControlCommand Update()
        {
            if (!Enabled || !_controller.IsInitialized || !_handle.IsAvailable)
                return null;

            // 获取当前末端执行器位姿
            var eePose = _controller.GetEndEffectorPose();
            var eePos = eePose.GetPosition();
            var eeRot = eePose.GetRotation();

            if (!_initialized)
            {
                _lastHandlePosition = eePos;
                _lastHandleRotation = eeRot;
                _initialized = true;
            }

            // 使用手柄 API 获取拖拽后的新值
            var newPos = _handle.PositionHandle(_lastHandlePosition, _lastHandleRotation);
            var newRot = _handle.RotationHandle(_lastHandleRotation, _lastHandlePosition);

            // 检测位置变化
            var posDelta = newPos - _lastHandlePosition;
            bool posChanged = posDelta.SqrMagnitude > 1e-10f;

            // 检测旋转变化
            var rotDiff = RMMathUtils.ComputeRotationError(newRot, _lastHandleRotation);
            bool rotChanged = rotDiff.SqrMagnitude > 1e-10f;

            _lastHandlePosition = newPos;
            _lastHandleRotation = newRot;

            if (posChanged || rotChanged)
            {
                // 直接设定绝对目标位姿
                return new EndEffectorMoveCommand(newPos, newRot);
            }

            return null;
        }

        /// <summary>
        /// 重置手柄到当前末端执行器位姿（当外部修改了关节角度后调用）。
        /// </summary>
        public void ResetHandleState()
        {
            _initialized = false;
        }
    }
}
