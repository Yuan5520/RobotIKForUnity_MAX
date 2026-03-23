using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Controller;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 鼠标输入配置。
    /// </summary>
    public class MouseInputConfig
    {
        /// <summary>拖拽使用的鼠标按钮索引（0=左键，1=右键，2=中键）。</summary>
        public int DragButton = 0;

        /// <summary>位置灵敏度缩放因子。</summary>
        public float PositionSensitivity = 0.01f;

        /// <summary>旋转灵敏度缩放因子（弧度/像素）。</summary>
        public float RotationSensitivity = 0.005f;

        /// <summary>是否在按住拖拽键时使用旋转模式（通过右键区分）。</summary>
        public int RotationButton = 1;

        /// <summary>
        /// 屏幕空间到世界空间的深度参考值。
        /// 鼠标射线投射到此深度平面获取世界坐标。
        /// </summary>
        public float ReferenceDepth = 5.0f;
    }

    /// <summary>
    /// 鼠标输入处理器。
    /// 通过鼠标拖拽实现末端执行器的位姿增量控制。
    /// 左键拖拽 = 位置移动，右键拖拽 = 旋转。
    /// </summary>
    public class MouseInputHandler : IInputHandler
    {
        private readonly IEngineInput _input;
        private readonly RobotArmController _controller;
        private readonly MouseInputConfig _config;

        private bool _isDragging;
        private bool _isRotating;
        private RMVector3 _lastMouseWorld;

        public bool Enabled { get; set; } = true;

        public MouseInputHandler(IEngineInput input, RobotArmController controller,
                                  MouseInputConfig config = null)
        {
            _input = input;
            _controller = controller;
            _config = config ?? new MouseInputConfig();
        }

        public IControlCommand Update()
        {
            if (!Enabled || !_controller.IsInitialized)
                return null;

            // 位置拖拽开始
            if (_input.GetMouseButtonDown(_config.DragButton))
            {
                _isDragging = true;
                _isRotating = false;
                _lastMouseWorld = _input.GetMouseWorldPosition(RMVector3.Zero, _config.ReferenceDepth);
                return null;
            }

            // 旋转拖拽开始
            if (_input.GetMouseButtonDown(_config.RotationButton))
            {
                _isRotating = true;
                _isDragging = false;
                _lastMouseWorld = _input.GetMouseWorldPosition(RMVector3.Zero, _config.ReferenceDepth);
                return null;
            }

            // 位置拖拽中
            if (_isDragging && _input.GetMouseButton(_config.DragButton))
            {
                var currentWorld = _input.GetMouseWorldPosition(RMVector3.Zero, _config.ReferenceDepth);
                var delta = (currentWorld - _lastMouseWorld) * _config.PositionSensitivity;
                _lastMouseWorld = currentWorld;

                if (delta.SqrMagnitude < 1e-12f)
                    return null;

                return new EndEffectorDeltaCommand(delta, RMVector3.Zero);
            }
            else
            {
                _isDragging = false;
            }

            // 旋转拖拽中
            if (_isRotating && _input.GetMouseButton(_config.RotationButton))
            {
                float mouseX = _input.GetAxis("Mouse X");
                float mouseY = _input.GetAxis("Mouse Y");

                if (System.Math.Abs(mouseX) < 1e-10f && System.Math.Abs(mouseY) < 1e-10f)
                    return null;

                // Mouse X -> 绕 Y 轴旋转, Mouse Y -> 绕 X 轴旋转
                var rotDelta = new RMVector3(
                    -mouseY * _config.RotationSensitivity,
                    mouseX * _config.RotationSensitivity,
                    0f
                );

                return new EndEffectorDeltaCommand(RMVector3.Zero, rotDelta);
            }
            else
            {
                _isRotating = false;
            }

            return null;
        }
    }
}
