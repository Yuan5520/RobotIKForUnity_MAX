using NUnit.Framework;
using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Kinematics;
using RobotMatrix.Controller;
using RobotMatrix.Interaction;

namespace RobotMatrix.Tests
{
    /// <summary>
    /// 端到端集成测试 — 验证从输入命令到末端位姿的完整流水线。
    /// 覆盖：命令 → Dispatcher → Controller → FK/IK → 场景同步。
    /// </summary>
    public class IntegrationTests
    {
        private const float Epsilon = 0.05f;
        private static readonly float Pi = RMMathUtils.PI;

        // ===== 辅助：创建 2R 平面臂的完整集成环境 =====
        private struct IntegrationSetup
        {
            public RobotArmController Controller;
            public CommandDispatcher Dispatcher;
            public InputManager InputMgr;
            public MockEngineInput MockInput;
            public KeyboardInputHandler KeyboardHandler;
            public MockEngineObject[] Joints;
        }

        private IntegrationSetup Create2RIntegration()
        {
            // 模型: 2R 平面臂，各连杆长度 1
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            // Mock 场景层级: Root → J0 → J1
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var joints = new MockEngineObject[] { j0, j1 };
            var engineJoints = new IEngineObject[] { j0, j1 };

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, engineJoints);

            var dispatcher = new CommandDispatcher(ctrl);
            var inputMgr = new InputManager(dispatcher);

            var mockInput = new MockEngineInput();
            var keyboardHandler = new KeyboardInputHandler(mockInput, ctrl);
            inputMgr.Register(keyboardHandler);

            return new IntegrationSetup
            {
                Controller = ctrl,
                Dispatcher = dispatcher,
                InputMgr = inputMgr,
                MockInput = mockInput,
                KeyboardHandler = keyboardHandler,
                Joints = joints
            };
        }

        // ===== 辅助：创建 3R Z-stacked 臂 =====
        private IntegrationSetup Create3RIntegration()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(0f, Pi / 2f, 0.5f, 0f),
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.LocalRotation = RMQuaternion.Identity;
            var j1 = new MockEngineObject("J1", j0);
            j1.LocalPosition = new RMVector3(0f, 0.5f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;
            var j2 = new MockEngineObject("J2", j1);
            j2.LocalPosition = new RMVector3(1f, 0f, 0f);
            j2.LocalRotation = RMQuaternion.Identity;

            var joints = new MockEngineObject[] { j0, j1, j2 };
            var engineJoints = new IEngineObject[] { j0, j1, j2 };

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, engineJoints);

            var dispatcher = new CommandDispatcher(ctrl);
            var inputMgr = new InputManager(dispatcher);

            var mockInput = new MockEngineInput();
            var keyboardHandler = new KeyboardInputHandler(mockInput, ctrl);
            inputMgr.Register(keyboardHandler);

            return new IntegrationSetup
            {
                Controller = ctrl,
                Dispatcher = dispatcher,
                InputMgr = inputMgr,
                MockInput = mockInput,
                KeyboardHandler = keyboardHandler,
                Joints = joints
            };
        }

        // =========================================================
        //  Test 1: FK 键盘控制 → 关节角度 → 末端位姿 完整管线
        // =========================================================

        [Test]
        public void E2E_FK_KeyboardRotateJoint_UpdatesEndEffector()
        {
            var setup = Create2RIntegration();

            // 模拟多帧按 d 键（正向旋转 J0）
            int frames = 50; // 50 * 0.02 rad = 1.0 rad ≈ 57°
            for (int f = 0; f < frames; f++)
            {
                setup.MockInput.SetKey("d", true);
                setup.InputMgr.Tick();
            }
            setup.MockInput.SetKey("d", false);

            // 验证 J0 角度约为 1.0 rad
            var angles = setup.Controller.GetJointAngles();
            Assert.AreEqual(1.0f, angles[0], 0.05f, "J0 angle should be ~1.0 rad");
            Assert.AreEqual(0f, angles[1], 0.001f, "J1 angle should remain 0");

            // 验证末端位置（2R 平面: θ1=1, θ2=0 → ee = (cos1+cos1, sin1+sin1, 0)）
            float c = (float)System.Math.Cos(angles[0]);
            float s = (float)System.Math.Sin(angles[0]);
            var ee = setup.Controller.GetEndEffectorPose().GetPosition();
            Assert.AreEqual(c + c, ee.X, Epsilon, "EE X");
            Assert.AreEqual(s + s, ee.Y, Epsilon, "EE Y");
        }

        // =========================================================
        //  Test 2: 关节选择 → 切换 → 旋转第二关节
        // =========================================================

        [Test]
        public void E2E_FK_SelectNextJoint_RotateSecondJoint()
        {
            var setup = Create2RIntegration();

            // 选择下一个关节 (J0 → J1)
            setup.MockInput.SetKeyDown("e");
            setup.InputMgr.Tick();
            Assert.AreEqual(1, setup.KeyboardHandler.SelectedJointIndex);

            // 旋转 J1
            setup.MockInput.ClearAll();
            for (int f = 0; f < 25; f++) // 25 * 0.02 = 0.5 rad
            {
                setup.MockInput.SetKey("d", true);
                setup.InputMgr.Tick();
            }

            var angles = setup.Controller.GetJointAngles();
            Assert.AreEqual(0f, angles[0], 0.001f, "J0 should be unchanged");
            Assert.AreEqual(0.5f, angles[1], 0.05f, "J1 should be ~0.5 rad");
        }

        // =========================================================
        //  Test 3: 模式切换 FK → IK → 增量移动
        // =========================================================

        [Test]
        public void E2E_ModeSwitch_ThenIKDelta()
        {
            var setup = Create2RIntegration();

            Assert.AreEqual(ControlMode.FK, setup.Controller.GetControlMode());

            // 切换到 IK 模式
            setup.MockInput.SetKeyDown("tab");
            setup.InputMgr.Tick();
            Assert.AreEqual(ControlMode.IK, setup.Controller.GetControlMode());

            // 记录初始末端位置
            var before = setup.Controller.GetEndEffectorPose().GetPosition();

            // 按 d 键在 IK 模式下增量移动 +X
            setup.MockInput.ClearAll();
            for (int f = 0; f < 20; f++)
            {
                setup.MockInput.SetKey("d", true);
                setup.InputMgr.Tick();
            }

            var after = setup.Controller.GetEndEffectorPose().GetPosition();

            // 末端应该在 X 方向有移动
            Assert.Greater(System.Math.Abs(after.X - before.X), 0.01f,
                "EE should have moved in X direction");
        }

        // =========================================================
        //  Test 4: 直接 Dispatch 命令 → IK 求解 → 末端到达
        // =========================================================

        [Test]
        public void E2E_DirectDispatch_IKMoveCommand_Converges()
        {
            var setup = Create2RIntegration();

            // 目标: (1, 1, 0) — 2R 平面臂可达点
            var target = new RMVector3(1f, 1f, 0f);
            setup.Dispatcher.Dispatch(
                new EndEffectorMoveCommand(target, RMQuaternion.Identity));

            var ee = setup.Controller.GetEndEffectorPose().GetPosition();
            float err = (ee - target).Magnitude;
            Assert.Less(err, 0.1f, $"EE should reach near target. Actual pos={ee}, err={err}");
        }

        // =========================================================
        //  Test 5: 3R 臂完整管线 — 多关节协调
        // =========================================================

        [Test]
        public void E2E_3R_MultiJointFK_ThenIK()
        {
            var setup = Create3RIntegration();

            // FK: 设置角度
            setup.Controller.SetJointAngles(new float[] { Pi / 4f, Pi / 4f, 0f });

            var afterFK = setup.Controller.GetEndEffectorPose().GetPosition();
            Assert.Greater(afterFK.Magnitude, 0.1f, "EE should be at non-zero position");

            // 记录 FK 后位置，然后 IK 回到原点附近
            var target = new RMVector3(1.5f, 0.5f, 0.5f);
            var result = setup.Controller.MoveEndEffectorTo(target, RMQuaternion.Identity);

            Assert.IsNotNull(result, "IK result should not be null");
            var afterIK = setup.Controller.GetEndEffectorPose().GetPosition();
            float dist = (afterIK - target).Magnitude;

            // 3R 臂在工作空间内应该能接近目标
            Assert.Less(dist, 0.5f, $"3R arm should approach target. err={dist}");
        }

        // =========================================================
        //  Test 6: 热更新不破坏工作流
        // =========================================================

        [Test]
        public void E2E_HotUpdate_PreservesAngles()
        {
            var setup = Create2RIntegration();

            // 设定角度
            setup.Controller.SetJointAngles(new float[] { 0.5f, 0.3f });
            var before = setup.Controller.GetEndEffectorPose().GetPosition();

            // 触发热更新
            setup.Controller.MarkDirty();
            setup.Controller.ProcessDirtyFlag();

            var after = setup.Controller.GetEndEffectorPose().GetPosition();
            Assert.AreEqual(before.X, after.X, Epsilon, "X preserved");
            Assert.AreEqual(before.Y, after.Y, Epsilon, "Y preserved");
        }

        // =========================================================
        //  Test 7: InputManager 优先级 — 多处理器协同
        // =========================================================

        [Test]
        public void E2E_InputManager_MultiHandler_Priority()
        {
            var setup = Create2RIntegration();

            // 创建第二个 mock handler，模拟鼠标
            var mouseInput = new MockEngineInput();
            var mouseHandler = new MouseInputHandler(mouseInput, setup.Controller);
            setup.InputMgr.Register(mouseHandler);

            // 键盘有命令，鼠标无命令 → 键盘命令应生效
            setup.MockInput.SetKey("d", true);
            setup.InputMgr.Tick();

            var angles = setup.Controller.GetJointAngles();
            Assert.Greater(angles[0], 0f, "Keyboard command should have executed");
        }

        // =========================================================
        //  Test 8: 关节限位在完整管线中生效
        // =========================================================

        [Test]
        public void E2E_JointLimits_EnforcedInPipeline()
        {
            var setup = Create2RIntegration();

            // 设置 J0 限位 [-30°, +30°]
            float limit = 30f * RMMathUtils.Deg2Rad;
            var cfg = setup.Controller.Model.JointConfigs[0];
            cfg.HasLimits = true;
            cfg.MinLimit = -limit;
            cfg.MaxLimit = limit;

            // 通过命令尝试设置超出限位的角度
            setup.Dispatcher.Dispatch(new JointAngleCommand(0, Pi)); // π >> 30°

            var angles = setup.Controller.GetJointAngles();
            Assert.AreEqual(limit, angles[0], 0.001f, "Should be clamped to max limit");
        }

        // =========================================================
        //  Test 9: Dispatcher + 批量关节角度命令
        // =========================================================

        [Test]
        public void E2E_Dispatcher_JointAnglesAll_ThenQueryEE()
        {
            var setup = Create2RIntegration();

            var targetAngles = new float[] { Pi / 2f, -Pi / 4f };
            setup.Dispatcher.Dispatch(new JointAnglesAllCommand(targetAngles));

            var angles = setup.Controller.GetJointAngles();
            Assert.AreEqual(Pi / 2f, angles[0], 0.001f);
            Assert.AreEqual(-Pi / 4f, angles[1], 0.001f);

            // 验证末端位姿正确性 (θ1=π/2, θ2=-π/4)
            var ee = setup.Controller.GetEndEffectorPose().GetPosition();
            // 第一连杆 (0,1,0), 第二连杆在 θ1+θ2 = π/4 方向
            float c12 = (float)System.Math.Cos(Pi / 2f - Pi / 4f);
            float s12 = (float)System.Math.Sin(Pi / 2f - Pi / 4f);
            float expectedX = 0f + c12; // cos(π/2) + cos(π/2 - π/4) = 0 + cos(π/4)
            float expectedY = 1f + s12; // sin(π/2) + sin(π/4)
            Assert.AreEqual(expectedX, ee.X, Epsilon, "EE X");
            Assert.AreEqual(expectedY, ee.Y, Epsilon, "EE Y");
        }
    }
}
