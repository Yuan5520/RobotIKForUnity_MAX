using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Controller;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 键盘输入配置 — 定义按键绑定与灵敏度。
    /// </summary>
    public class KeyboardInputConfig
    {
        // ===== FK 模式：关节选择与旋转 =====

        /// <summary>选择上一个关节的按键。</summary>
        public string PrevJointKey = "q";

        /// <summary>选择下一个关节的按键。</summary>
        public string NextJointKey = "e";

        /// <summary>关节正向旋转的按键。</summary>
        public string JointPositiveKey = "d";

        /// <summary>关节负向旋转的按键。</summary>
        public string JointNegativeKey = "a";

        /// <summary>关节角度变化速率（弧度/帧）。</summary>
        public float JointAngleStep = 0.02f;

        // ===== IK 模式：末端执行器增量移动 =====

        /// <summary>+X 方向移动按键。</summary>
        public string MoveXPosKey = "d";

        /// <summary>-X 方向移动按键。</summary>
        public string MoveXNegKey = "a";

        /// <summary>+Y 方向移动按键。</summary>
        public string MoveYPosKey = "space";

        /// <summary>-Y 方向移动按键。</summary>
        public string MoveYNegKey = "left ctrl";

        /// <summary>+Z 方向移动按键。</summary>
        public string MoveZPosKey = "w";

        /// <summary>-Z 方向移动按键。</summary>
        public string MoveZNegKey = "s";

        /// <summary>位置增量步长（米/帧）。</summary>
        public float PositionStep = 0.005f;

        // ===== IK 模式：末端执行器增量旋转 =====

        /// <summary>绕 X 轴正向旋转（俯仰+）。</summary>
        public string RotXPosKey = "i";

        /// <summary>绕 X 轴负向旋转（俯仰-）。</summary>
        public string RotXNegKey = "k";

        /// <summary>绕 Y 轴正向旋转（偏航+）。</summary>
        public string RotYPosKey = "l";

        /// <summary>绕 Y 轴负向旋转（偏航-）。</summary>
        public string RotYNegKey = "j";

        /// <summary>绕 Z 轴正向旋转（翻滚+）。</summary>
        public string RotZPosKey = "u";

        /// <summary>绕 Z 轴负向旋转（翻滚-）。</summary>
        public string RotZNegKey = "o";

        /// <summary>旋转增量步长（弧度/帧）。</summary>
        public float RotationStep = 0.02f;

        // ===== 模式切换 =====

        /// <summary>切换 FK/IK 模式的按键。</summary>
        public string ToggleModeKey = "tab";
    }

    /// <summary>
    /// 键盘输入处理器。
    /// 通过 IEngineInput 读取按键状态，根据当前控制模式生成对应的 IControlCommand。
    /// FK 模式：选中关节 + 增量旋转；IK 模式：末端执行器增量平移。
    /// </summary>
    public class KeyboardInputHandler : IInputHandler
    {
        private readonly IEngineInput _input;
        private readonly RobotArmController _controller;
        private readonly KeyboardInputConfig _config;

        private int _selectedJointIndex;

        public bool Enabled { get; set; } = true;

        /// <summary>当前选中的关节索引。</summary>
        public int SelectedJointIndex
        {
            get => _selectedJointIndex;
            set
            {
                if (_controller.IsInitialized && _controller.DOF > 0)
                    _selectedJointIndex = RMMathUtils.Clamp(value, 0, _controller.DOF - 1);
            }
        }

        public KeyboardInputHandler(IEngineInput input, RobotArmController controller,
                                     KeyboardInputConfig config = null)
        {
            _input = input;
            _controller = controller;
            _config = config ?? new KeyboardInputConfig();
            _selectedJointIndex = 0;
        }

        public IControlCommand Update()
        {
            if (!Enabled || !_controller.IsInitialized)
                return null;

            // 模式切换优先
            if (_input.GetKeyDown(_config.ToggleModeKey))
            {
                var currentMode = _controller.GetControlMode();
                var newMode = currentMode == ControlMode.FK ? ControlMode.IK : ControlMode.FK;
                return new ModeSwitchCommand(newMode);
            }

            if (_controller.GetControlMode() == ControlMode.FK)
                return UpdateFK();
            else
                return UpdateIK();
        }

        private IControlCommand UpdateFK()
        {
            int dof = _controller.DOF;
            if (dof <= 0) return null;

            // 关节选择
            if (_input.GetKeyDown(_config.PrevJointKey))
            {
                _selectedJointIndex = (_selectedJointIndex - 1 + dof) % dof;
                return null; // 选择操作本身不生成运动命令
            }
            if (_input.GetKeyDown(_config.NextJointKey))
            {
                _selectedJointIndex = (_selectedJointIndex + 1) % dof;
                return null;
            }

            // 关节旋转增量
            float delta = 0f;
            if (_input.GetKey(_config.JointPositiveKey))
                delta += _config.JointAngleStep;
            if (_input.GetKey(_config.JointNegativeKey))
                delta -= _config.JointAngleStep;

            if (System.Math.Abs(delta) < 1e-10f)
                return null;

            var currentAngles = _controller.GetJointAngles();
            float newAngle = currentAngles[_selectedJointIndex] + delta;

            // Shift + A/D：拖动关节保持 TCP 位置不变（零空间交互）
            if (_input.GetKey("left shift") || _input.GetKey("right shift"))
            {
                return new JointDragKeepTCPCommand(_selectedJointIndex, newAngle);
            }

            return new JointAngleCommand(_selectedJointIndex, newAngle);
        }

        private IControlCommand UpdateIK()
        {
            // 位置增量
            float px = 0f, py = 0f, pz = 0f;

            if (_input.GetKey(_config.MoveXPosKey)) px += _config.PositionStep;
            if (_input.GetKey(_config.MoveXNegKey)) px -= _config.PositionStep;
            if (_input.GetKey(_config.MoveYPosKey)) py += _config.PositionStep;
            if (_input.GetKey(_config.MoveYNegKey)) py -= _config.PositionStep;
            if (_input.GetKey(_config.MoveZPosKey)) pz += _config.PositionStep;
            if (_input.GetKey(_config.MoveZNegKey)) pz -= _config.PositionStep;

            // 旋转增量
            float rx = 0f, ry = 0f, rz = 0f;

            if (_input.GetKey(_config.RotXPosKey)) rx += _config.RotationStep;
            if (_input.GetKey(_config.RotXNegKey)) rx -= _config.RotationStep;
            if (_input.GetKey(_config.RotYPosKey)) ry += _config.RotationStep;
            if (_input.GetKey(_config.RotYNegKey)) ry -= _config.RotationStep;
            if (_input.GetKey(_config.RotZPosKey)) rz += _config.RotationStep;
            if (_input.GetKey(_config.RotZNegKey)) rz -= _config.RotationStep;

            bool hasPos = System.Math.Abs(px) > 1e-10f ||
                          System.Math.Abs(py) > 1e-10f ||
                          System.Math.Abs(pz) > 1e-10f;
            bool hasRot = System.Math.Abs(rx) > 1e-10f ||
                          System.Math.Abs(ry) > 1e-10f ||
                          System.Math.Abs(rz) > 1e-10f;

            if (!hasPos && !hasRot)
                return null;

            var posDelta = new RMVector3(px, py, pz);
            var rotDelta = new RMVector3(rx, ry, rz);
            return new EndEffectorDeltaCommand(posDelta, rotDelta);
        }
    }
}
