using NUnit.Framework;
using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Kinematics;
using RobotMatrix.Controller;
using RobotMatrix.Interaction;

namespace RobotMatrix.Tests
{
    /// <summary>
    /// Mock IEngineInput，用于测试输入处理器。
    /// </summary>
    public class MockEngineInput : IEngineInput
    {
        // 按键状态字典
        private readonly System.Collections.Generic.Dictionary<string, bool> _keys
            = new System.Collections.Generic.Dictionary<string, bool>();
        private readonly System.Collections.Generic.Dictionary<string, bool> _keysDown
            = new System.Collections.Generic.Dictionary<string, bool>();
        private readonly System.Collections.Generic.Dictionary<int, bool> _mouseButtons
            = new System.Collections.Generic.Dictionary<int, bool>();
        private readonly System.Collections.Generic.Dictionary<int, bool> _mouseButtonsDown
            = new System.Collections.Generic.Dictionary<int, bool>();
        private readonly System.Collections.Generic.Dictionary<string, float> _axes
            = new System.Collections.Generic.Dictionary<string, float>();

        private RMVector3 _mouseWorldPos;

        public void SetKey(string keyName, bool held, bool down = false)
        {
            _keys[keyName] = held;
            _keysDown[keyName] = down;
        }

        public void SetMouseButton(int button, bool held, bool down = false)
        {
            _mouseButtons[button] = held;
            _mouseButtonsDown[button] = down;
        }

        public void SetAxis(string axisName, float value)
        {
            _axes[axisName] = value;
        }

        public void SetMouseWorldPosition(RMVector3 pos)
        {
            _mouseWorldPos = pos;
        }

        /// <summary>
        /// 模拟按键按下瞬间（GetKeyDown=true，GetKey=true）。
        /// </summary>
        public void SetKeyDown(string keyName)
        {
            _keys[keyName] = true;
            _keysDown[keyName] = true;
        }

        public void ClearAll()
        {
            _keys.Clear();
            _keysDown.Clear();
            _mouseButtons.Clear();
            _mouseButtonsDown.Clear();
            _axes.Clear();
        }

        // IEngineInput 实现
        public RMVector3 GetMouseWorldPosition(RMVector3 screenPos, float depth) => _mouseWorldPos;
        public bool GetMouseButton(int button) =>
            _mouseButtons.ContainsKey(button) && _mouseButtons[button];
        public bool GetMouseButtonDown(int button) =>
            _mouseButtonsDown.ContainsKey(button) && _mouseButtonsDown[button];
        public bool GetKey(string keyName) =>
            _keys.ContainsKey(keyName) && _keys[keyName];
        public bool GetKeyDown(string keyName) =>
            _keysDown.ContainsKey(keyName) && _keysDown[keyName];
        public float GetAxis(string axisName) =>
            _axes.ContainsKey(axisName) ? _axes[axisName] : 0f;
    }

    /// <summary>
    /// Mock IEngineHandle，用于测试编辑器手柄处理器。
    /// </summary>
    public class MockEngineHandle : IEngineHandle
    {
        public bool IsAvailable { get; set; } = true;

        private RMVector3 _positionOverride;
        private RMQuaternion _rotationOverride;
        private bool _hasPositionOverride;
        private bool _hasRotationOverride;

        /// <summary>
        /// 设置下一次 PositionHandle 调用返回的位置（模拟拖拽）。
        /// </summary>
        public void SimulateDragPosition(RMVector3 newPos)
        {
            _positionOverride = newPos;
            _hasPositionOverride = true;
        }

        /// <summary>
        /// 设置下一次 RotationHandle 调用返回的旋转（模拟拖拽）。
        /// </summary>
        public void SimulateDragRotation(RMQuaternion newRot)
        {
            _rotationOverride = newRot;
            _hasRotationOverride = true;
        }

        public void ClearOverrides()
        {
            _hasPositionOverride = false;
            _hasRotationOverride = false;
        }

        public RMVector3 PositionHandle(RMVector3 position, RMQuaternion rotation)
        {
            if (_hasPositionOverride)
            {
                _hasPositionOverride = false;
                return _positionOverride;
            }
            return position;
        }

        public RMQuaternion RotationHandle(RMQuaternion rotation, RMVector3 position)
        {
            if (_hasRotationOverride)
            {
                _hasRotationOverride = false;
                return _rotationOverride;
            }
            return rotation;
        }
    }

    // =========================================================================
    //  命令类测试
    // =========================================================================

    public class ControlCommandTests
    {
        [Test]
        public void EndEffectorMoveCommand_StoresValues()
        {
            var pos = new RMVector3(1f, 2f, 3f);
            var rot = RMQuaternion.Identity;
            var cmd = new EndEffectorMoveCommand(pos, rot);

            Assert.AreEqual(ControlCommandType.EndEffectorMove, cmd.Type);
            Assert.AreEqual(1f, cmd.TargetPosition.X, 1e-6f);
            Assert.AreEqual(2f, cmd.TargetPosition.Y, 1e-6f);
            Assert.AreEqual(3f, cmd.TargetPosition.Z, 1e-6f);
        }

        [Test]
        public void EndEffectorDeltaCommand_StoresValues()
        {
            var posDelta = new RMVector3(0.1f, 0.2f, 0f);
            var rotDelta = new RMVector3(0f, 0f, 0.5f);
            var cmd = new EndEffectorDeltaCommand(posDelta, rotDelta);

            Assert.AreEqual(ControlCommandType.EndEffectorDelta, cmd.Type);
            Assert.AreEqual(0.1f, cmd.PositionDelta.X, 1e-6f);
            Assert.AreEqual(0.5f, cmd.RotationDelta.Z, 1e-6f);
        }

        [Test]
        public void JointAngleCommand_StoresValues()
        {
            var cmd = new JointAngleCommand(2, 1.5f);

            Assert.AreEqual(ControlCommandType.JointAngle, cmd.Type);
            Assert.AreEqual(2, cmd.JointIndex);
            Assert.AreEqual(1.5f, cmd.Angle, 1e-6f);
        }

        [Test]
        public void JointAnglesAllCommand_StoresValues()
        {
            var angles = new float[] { 0.1f, 0.2f, 0.3f };
            var cmd = new JointAnglesAllCommand(angles);

            Assert.AreEqual(ControlCommandType.JointAnglesAll, cmd.Type);
            Assert.AreEqual(3, cmd.Angles.Length);
            Assert.AreEqual(0.2f, cmd.Angles[1], 1e-6f);
        }

        [Test]
        public void ModeSwitchCommand_StoresMode()
        {
            var cmd = new ModeSwitchCommand(ControlMode.IK);
            Assert.AreEqual(ControlCommandType.ModeSwitch, cmd.Type);
            Assert.AreEqual(ControlMode.IK, cmd.TargetMode);
        }
    }

    // =========================================================================
    //  CommandDispatcher 测试
    // =========================================================================

    public class CommandDispatcherTests
    {
        private RobotArmController CreateInitializedController()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, new IEngineObject[] { j0, j1 });
            return ctrl;
        }

        [Test]
        public void Dispatch_NullCommand_ReturnsFalse()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            Assert.IsFalse(dispatcher.Dispatch(null));
        }

        [Test]
        public void Dispatch_JointAngleCommand_UpdatesAngle()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            var cmd = new JointAngleCommand(0, 0.5f);
            bool result = dispatcher.Dispatch(cmd);

            Assert.IsTrue(result);
            Assert.AreEqual(0.5f, ctrl.GetJointAngles()[0], 1e-5f);
        }

        [Test]
        public void Dispatch_JointAnglesAllCommand_UpdatesAll()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            var cmd = new JointAnglesAllCommand(new float[] { 0.3f, 0.4f });
            dispatcher.Dispatch(cmd);

            var angles = ctrl.GetJointAngles();
            Assert.AreEqual(0.3f, angles[0], 1e-5f);
            Assert.AreEqual(0.4f, angles[1], 1e-5f);
        }

        [Test]
        public void Dispatch_ModeSwitchCommand_SwitchesMode()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            Assert.AreEqual(ControlMode.FK, ctrl.GetControlMode());

            dispatcher.Dispatch(new ModeSwitchCommand(ControlMode.IK));
            Assert.AreEqual(ControlMode.IK, ctrl.GetControlMode());
        }

        [Test]
        public void Dispatch_EndEffectorDeltaCommand_MovesEE()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            var before = ctrl.GetEndEffectorPose().GetPosition();
            var cmd = new EndEffectorDeltaCommand(new RMVector3(0.1f, 0f, 0f), RMVector3.Zero);
            dispatcher.Dispatch(cmd);
            var after = ctrl.GetEndEffectorPose().GetPosition();

            // 末端位置应有变化
            float diff = (after - before).Magnitude;
            Assert.Greater(diff, 0.001f, "EE should have moved");
        }

        [Test]
        public void Dispatch_EndEffectorMoveCommand_MovesEE()
        {
            var ctrl = CreateInitializedController();
            var dispatcher = new CommandDispatcher(ctrl);

            var cmd = new EndEffectorMoveCommand(
                new RMVector3(1f, 1f, 0f), RMQuaternion.Identity);
            bool result = dispatcher.Dispatch(cmd);

            Assert.IsTrue(result);
            var pos = ctrl.GetEndEffectorPose().GetPosition();
            float err = (pos - new RMVector3(1f, 1f, 0f)).Magnitude;
            Assert.Less(err, 0.15f, "EE should be near target");
        }
    }

    // =========================================================================
    //  KeyboardInputHandler 测试
    // =========================================================================

    public class KeyboardInputHandlerTests
    {
        private (KeyboardInputHandler handler, MockEngineInput input, RobotArmController ctrl)
            CreateSetup()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, new IEngineObject[] { j0, j1 });

            var input = new MockEngineInput();
            var handler = new KeyboardInputHandler(input, ctrl);

            return (handler, input, ctrl);
        }

        [Test]
        public void Update_NoInput_ReturnsNull()
        {
            var (handler, _, _) = CreateSetup();
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_Disabled_ReturnsNull()
        {
            var (handler, input, _) = CreateSetup();
            handler.Enabled = false;
            input.SetKey("d", true);
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_FK_JointPositive_ReturnsJointAngleCommand()
        {
            var (handler, input, ctrl) = CreateSetup();

            // 默认 FK 模式，按 d 键正向旋转
            input.SetKey("d", true);
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            Assert.AreEqual(ControlCommandType.JointAngle, cmd.Type);
            var jCmd = (JointAngleCommand)cmd;
            Assert.AreEqual(0, jCmd.JointIndex);
            Assert.Greater(jCmd.Angle, 0f, "Angle should be positive");
        }

        [Test]
        public void Update_FK_JointNegative_ReturnsNegativeAngle()
        {
            var (handler, input, _) = CreateSetup();

            input.SetKey("a", true);
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            var jCmd = (JointAngleCommand)cmd;
            Assert.Less(jCmd.Angle, 0f, "Angle should be negative");
        }

        [Test]
        public void Update_FK_NextJoint_CyclesSelection()
        {
            var (handler, input, _) = CreateSetup();

            Assert.AreEqual(0, handler.SelectedJointIndex);

            // 按 e 选择下一关节
            input.SetKeyDown("e");
            handler.Update();
            Assert.AreEqual(1, handler.SelectedJointIndex);

            // 再按一次回到 0（循环）
            input.SetKeyDown("e");
            handler.Update();
            Assert.AreEqual(0, handler.SelectedJointIndex);
        }

        [Test]
        public void Update_FK_PrevJoint_CyclesBackward()
        {
            var (handler, input, _) = CreateSetup();

            Assert.AreEqual(0, handler.SelectedJointIndex);

            // 按 q 选择上一关节（循环到末尾）
            input.SetKeyDown("q");
            handler.Update();
            Assert.AreEqual(1, handler.SelectedJointIndex);
        }

        [Test]
        public void Update_ToggleMode_ReturnsModeSwitchCommand()
        {
            var (handler, input, ctrl) = CreateSetup();

            input.SetKeyDown("tab");
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            Assert.AreEqual(ControlCommandType.ModeSwitch, cmd.Type);
            var modeCmd = (ModeSwitchCommand)cmd;
            Assert.AreEqual(ControlMode.IK, modeCmd.TargetMode);
        }

        [Test]
        public void Update_IK_MovePosX_ReturnsDeltaCommand()
        {
            var (handler, input, ctrl) = CreateSetup();

            ctrl.SetControlMode(ControlMode.IK);

            input.SetKey("d", true);
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            Assert.AreEqual(ControlCommandType.EndEffectorDelta, cmd.Type);
            var deltaCmd = (EndEffectorDeltaCommand)cmd;
            Assert.Greater(deltaCmd.PositionDelta.X, 0f, "X delta should be positive");
            Assert.AreEqual(0f, deltaCmd.PositionDelta.Y, 1e-10f, "Y delta should be zero");
            Assert.AreEqual(0f, deltaCmd.PositionDelta.Z, 1e-10f, "Z delta should be zero");
        }

        [Test]
        public void Update_IK_MoveNegZ_ReturnsDeltaCommand()
        {
            var (handler, input, ctrl) = CreateSetup();

            ctrl.SetControlMode(ControlMode.IK);

            input.SetKey("s", true);
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            var deltaCmd = (EndEffectorDeltaCommand)cmd;
            Assert.Less(deltaCmd.PositionDelta.Z, 0f, "Z delta should be negative");
        }

        [Test]
        public void SelectedJointIndex_ClampedToRange()
        {
            var (handler, _, _) = CreateSetup();

            handler.SelectedJointIndex = 100;
            Assert.AreEqual(1, handler.SelectedJointIndex, "Should clamp to DOF-1");

            handler.SelectedJointIndex = -5;
            Assert.AreEqual(0, handler.SelectedJointIndex, "Should clamp to 0");
        }
    }

    // =========================================================================
    //  MouseInputHandler 测试
    // =========================================================================

    public class MouseInputHandlerTests
    {
        private (MouseInputHandler handler, MockEngineInput input, RobotArmController ctrl)
            CreateSetup()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, new IEngineObject[] { j0, j1 });

            var input = new MockEngineInput();
            var handler = new MouseInputHandler(input, ctrl);

            return (handler, input, ctrl);
        }

        [Test]
        public void Update_NoInput_ReturnsNull()
        {
            var (handler, _, _) = CreateSetup();
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_Disabled_ReturnsNull()
        {
            var (handler, input, _) = CreateSetup();
            handler.Enabled = false;
            input.SetMouseButton(0, false, true);
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_LeftButtonDown_StartsPositionDrag()
        {
            var (handler, input, _) = CreateSetup();

            // 帧 1: 按下左键 → 开始拖拽
            input.SetMouseButton(0, false, true);
            input.SetMouseWorldPosition(new RMVector3(1f, 0f, 0f));
            var cmd1 = handler.Update();
            Assert.IsNull(cmd1, "First frame should capture start pos, no command");

            // 帧 2: 持续拖拽，鼠标移动
            input.SetMouseButton(0, true, false);
            input.SetMouseWorldPosition(new RMVector3(2f, 0f, 0f));
            var cmd2 = handler.Update();
            Assert.IsNotNull(cmd2, "Should produce delta command");
            Assert.AreEqual(ControlCommandType.EndEffectorDelta, cmd2.Type);
        }

        [Test]
        public void Update_RightButtonDown_StartsRotationDrag()
        {
            var (handler, input, _) = CreateSetup();

            // 帧 1: 按下右键 → 开始旋转拖拽
            input.SetMouseButton(1, false, true);
            input.SetMouseWorldPosition(new RMVector3(0f, 0f, 0f));
            var cmd1 = handler.Update();
            Assert.IsNull(cmd1, "First frame should capture start, no command");

            // 帧 2: 持续拖拽
            input.SetMouseButton(1, true, false);
            input.SetAxis("Mouse X", 5f);
            input.SetAxis("Mouse Y", 0f);
            var cmd2 = handler.Update();
            Assert.IsNotNull(cmd2);
            Assert.AreEqual(ControlCommandType.EndEffectorDelta, cmd2.Type);
            var deltaCmd = (EndEffectorDeltaCommand)cmd2;
            Assert.AreEqual(0f, deltaCmd.PositionDelta.X, 1e-10f, "Position should be zero in rotation mode");
            Assert.Greater(System.Math.Abs(deltaCmd.RotationDelta.Y), 0f, "Should have Y rotation delta");
        }

        [Test]
        public void Update_ReleaseButton_StopsDrag()
        {
            var (handler, input, _) = CreateSetup();

            // 帧 1: 按下
            input.SetMouseButton(0, false, true);
            input.SetMouseWorldPosition(new RMVector3(0f, 0f, 0f));
            handler.Update();

            // 帧 2: 松开 (held = false)
            input.SetMouseButton(0, false, false);
            input.SetMouseWorldPosition(new RMVector3(5f, 0f, 0f));
            var cmd = handler.Update();
            Assert.IsNull(cmd, "Should not produce command after release");
        }
    }

    // =========================================================================
    //  HandleInputHandler 测试
    // =========================================================================

    public class HandleInputHandlerTests
    {
        private (HandleInputHandler handler, MockEngineHandle handle, RobotArmController ctrl)
            CreateSetup()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, new IEngineObject[] { j0, j1 });

            var handle = new MockEngineHandle();
            var handler = new HandleInputHandler(handle, ctrl);

            return (handler, handle, ctrl);
        }

        [Test]
        public void Update_HandleNotAvailable_ReturnsNull()
        {
            var (handler, handle, _) = CreateSetup();
            handle.IsAvailable = false;
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_Disabled_ReturnsNull()
        {
            var (handler, _, _) = CreateSetup();
            handler.Enabled = false;
            Assert.IsNull(handler.Update());
        }

        [Test]
        public void Update_NoHandleDrag_ReturnsNull()
        {
            var (handler, _, _) = CreateSetup();

            // 第一帧初始化
            handler.Update();
            // 第二帧无拖拽
            var cmd = handler.Update();
            Assert.IsNull(cmd, "No drag, no command");
        }

        [Test]
        public void Update_PositionDrag_ReturnsEndEffectorMoveCommand()
        {
            var (handler, handle, _) = CreateSetup();

            // 第一帧初始化手柄
            handler.Update();

            // 第二帧模拟拖拽位置
            handle.SimulateDragPosition(new RMVector3(1.5f, 0.5f, 0f));
            var cmd = handler.Update();

            Assert.IsNotNull(cmd);
            Assert.AreEqual(ControlCommandType.EndEffectorMove, cmd.Type);
            var moveCmd = (EndEffectorMoveCommand)cmd;
            Assert.AreEqual(1.5f, moveCmd.TargetPosition.X, 1e-4f);
            Assert.AreEqual(0.5f, moveCmd.TargetPosition.Y, 1e-4f);
        }

        [Test]
        public void ResetHandleState_ResetsInitialized()
        {
            var (handler, handle, _) = CreateSetup();

            // 初始化 + 一帧
            handler.Update();

            // 重置
            handler.ResetHandleState();

            // 下一帧应重新初始化（不生成命令）
            var cmd = handler.Update();
            Assert.IsNull(cmd, "After reset, first update should re-initialize");
        }
    }

    // =========================================================================
    //  InputManager 测试
    // =========================================================================

    public class InputManagerTests
    {
        /// <summary>
        /// 简单的 Mock 输入处理器，可预设返回的命令。
        /// </summary>
        private class SimpleTestHandler : IInputHandler
        {
            public bool Enabled { get; set; } = true;
            public IControlCommand NextCommand { get; set; }

            public IControlCommand Update()
            {
                if (!Enabled) return null;
                var cmd = NextCommand;
                NextCommand = null;
                return cmd;
            }
        }

        private (InputManager mgr, RobotArmController ctrl) CreateSetup()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, new IEngineObject[] { j0, j1 });

            var dispatcher = new CommandDispatcher(ctrl);
            var mgr = new InputManager(dispatcher);

            return (mgr, ctrl);
        }

        [Test]
        public void Register_IncreasesHandlerCount()
        {
            var (mgr, _) = CreateSetup();
            Assert.AreEqual(0, mgr.HandlerCount);

            var h1 = new SimpleTestHandler();
            mgr.Register(h1);
            Assert.AreEqual(1, mgr.HandlerCount);

            var h2 = new SimpleTestHandler();
            mgr.Register(h2);
            Assert.AreEqual(2, mgr.HandlerCount);
        }

        [Test]
        public void Register_DuplicateIgnored()
        {
            var (mgr, _) = CreateSetup();
            var h1 = new SimpleTestHandler();
            mgr.Register(h1);
            mgr.Register(h1); // 重复
            Assert.AreEqual(1, mgr.HandlerCount);
        }

        [Test]
        public void Unregister_RemovesHandler()
        {
            var (mgr, _) = CreateSetup();
            var h1 = new SimpleTestHandler();
            mgr.Register(h1);
            mgr.Unregister(h1);
            Assert.AreEqual(0, mgr.HandlerCount);
        }

        [Test]
        public void Tick_NoCommands_ReturnsFalse()
        {
            var (mgr, _) = CreateSetup();
            var h1 = new SimpleTestHandler();
            mgr.Register(h1);
            Assert.IsFalse(mgr.Tick());
        }

        [Test]
        public void Tick_ExecutesFirstNonNullCommand()
        {
            var (mgr, ctrl) = CreateSetup();

            var h1 = new SimpleTestHandler();
            var h2 = new SimpleTestHandler();
            mgr.Register(h1);
            mgr.Register(h2);

            // h1 无命令，h2 有命令
            h2.NextCommand = new JointAngleCommand(0, 0.7f);
            bool result = mgr.Tick();

            Assert.IsTrue(result);
            Assert.AreEqual(0.7f, ctrl.GetJointAngles()[0], 1e-5f);
        }

        [Test]
        public void Tick_HighPriorityHandlerWins()
        {
            var (mgr, ctrl) = CreateSetup();

            var hHigh = new SimpleTestHandler();
            var hLow = new SimpleTestHandler();
            mgr.Register(hHigh); // 先注册 = 高优先级
            mgr.Register(hLow);

            // 两个都有命令，高优先级赢
            hHigh.NextCommand = new JointAngleCommand(0, 0.5f);
            hLow.NextCommand = new JointAngleCommand(0, 0.9f);
            mgr.Tick();

            Assert.AreEqual(0.5f, ctrl.GetJointAngles()[0], 1e-5f);
        }

        [Test]
        public void Tick_DisabledHandler_Skipped()
        {
            var (mgr, ctrl) = CreateSetup();

            var hDisabled = new SimpleTestHandler { Enabled = false };
            var hEnabled = new SimpleTestHandler();
            mgr.Register(hDisabled);
            mgr.Register(hEnabled);

            hDisabled.NextCommand = new JointAngleCommand(0, 0.1f);
            hEnabled.NextCommand = new JointAngleCommand(0, 0.8f);
            mgr.Tick();

            Assert.AreEqual(0.8f, ctrl.GetJointAngles()[0], 1e-5f);
        }
    }
}
