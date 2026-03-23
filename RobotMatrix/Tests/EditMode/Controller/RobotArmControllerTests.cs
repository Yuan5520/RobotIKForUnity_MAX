using NUnit.Framework;
using RobotMatrix.Math;
using IEngine;
using RobotMatrix.Kinematics;
using RobotMatrix.Controller;

namespace RobotMatrix.Tests
{
    public class RobotArmControllerTests
    {
        private const float Epsilon = 0.02f;
        private static readonly float Pi = RMMathUtils.PI;

        /// <summary>
        /// 创建一条 2R 平面臂的模型 + Mock 场景对象。
        /// J0 在原点，J1 在 (1,0,0)，都绕 Z 轴旋转。
        /// 关节严格父子链: Root → J0 → J1。
        /// </summary>
        private (RobotArmController ctrl, MockEngineObject[] joints) Create2RSetup()
        {
            // 模型
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            // Mock 场景层级: Root → J0 → J1
            var root = new MockEngineObject("Root");
            var j0 = new MockEngineObject("J0", root);
            j0.WorldPosition = new RMVector3(0f, 0f, 0f);
            j0.LocalPosition = new RMVector3(0f, 0f, 0f);
            j0.LocalRotation = RMQuaternion.Identity;

            var j1 = new MockEngineObject("J1", j0);
            j1.WorldPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalPosition = new RMVector3(1f, 0f, 0f);
            j1.LocalRotation = RMQuaternion.Identity;

            var joints = new MockEngineObject[] { j0, j1 };
            var engineJoints = new IEngineObject[] { j0, j1 };

            var ctrl = new RobotArmController();
            ctrl.Initialize(model, engineJoints);

            return (ctrl, joints);
        }

        // ===== 初始化 =====

        [Test]
        public void Initialize_SetsUpCorrectly()
        {
            var (ctrl, _) = Create2RSetup();

            Assert.IsTrue(ctrl.IsInitialized);
            Assert.AreEqual(2, ctrl.DOF);
            Assert.AreEqual(ControlMode.FK, ctrl.GetControlMode());
        }

        // ===== FK 操作 =====

        [Test]
        public void SetJointAngle_UpdatesEndEffector()
        {
            var (ctrl, _) = Create2RSetup();

            ctrl.SetJointAngle(0, Pi / 2f);

            var ee = ctrl.GetEndEffectorPose();
            var pos = ee.GetPosition();

            // θ1=π/2, θ2=0 → 末端在 (0, 2, 0)
            Assert.AreEqual(0f, pos.X, Epsilon, "X");
            Assert.AreEqual(2f, pos.Y, Epsilon, "Y");
        }

        [Test]
        public void SetJointAngles_UpdatesAll()
        {
            var (ctrl, _) = Create2RSetup();

            ctrl.SetJointAngles(new float[] { Pi / 2f, Pi / 2f });

            var ee = ctrl.GetEndEffectorPose();
            var pos = ee.GetPosition();

            // θ1=π/2, θ2=π/2 → 末端在 (-1, 1, 0)
            Assert.AreEqual(-1f, pos.X, Epsilon, "X");
            Assert.AreEqual(1f, pos.Y, Epsilon, "Y");
        }

        [Test]
        public void GetJointAngles_ReturnsCopy()
        {
            var (ctrl, _) = Create2RSetup();
            ctrl.SetJointAngles(new float[] { 0.5f, 0.3f });

            var angles = ctrl.GetJointAngles();
            Assert.AreEqual(0.5f, angles[0], 1e-6f);
            Assert.AreEqual(0.3f, angles[1], 1e-6f);

            // 修改返回值不影响内部状态
            angles[0] = 999f;
            var angles2 = ctrl.GetJointAngles();
            Assert.AreEqual(0.5f, angles2[0], 1e-6f);
        }

        // ===== 关节限位 =====

        [Test]
        public void SetJointAngle_ClampsToLimits()
        {
            var (ctrl, _) = Create2RSetup();

            ctrl.Model.JointConfigs[0].HasLimits = true;
            ctrl.Model.JointConfigs[0].MinLimit = -Pi / 4f;
            ctrl.Model.JointConfigs[0].MaxLimit = Pi / 4f;

            ctrl.SetJointAngle(0, Pi);  // 超出上限

            var angles = ctrl.GetJointAngles();
            Assert.AreEqual(Pi / 4f, angles[0], 1e-5f, "Should be clamped to max");
        }

        // ===== IK 操作 =====

        [Test]
        public void MoveEndEffectorTo_ReachableTarget()
        {
            var (ctrl, _) = Create2RSetup();

            // 目标: (1, 1, 0) — 2R 平面臂可达
            var result = ctrl.MoveEndEffectorTo(
                new RMVector3(1f, 1f, 0f),
                RMQuaternion.Identity);

            Assert.IsNotNull(result);

            // 验证末端到达目标附近
            var ee = ctrl.GetEndEffectorPose();
            var pos = ee.GetPosition();
            float err = (pos - new RMVector3(1f, 1f, 0f)).Magnitude;
            Assert.Less(err, 0.1f, "End effector should be near target");
        }

        [Test]
        public void MoveEndEffectorDelta_MovesByDelta()
        {
            var (ctrl, _) = Create2RSetup();

            // 先设置一个初始姿态
            ctrl.SetJointAngles(new float[] { 0.3f, 0.2f });
            var before = ctrl.GetEndEffectorPose().GetPosition();

            // 增量移动
            ctrl.MoveEndEffectorDelta(
                new RMVector3(0.1f, 0f, 0f),
                RMVector3.Zero);

            var after = ctrl.GetEndEffectorPose().GetPosition();

            // 末端应该在 X 方向有所移动
            float dx = after.X - before.X;
            Assert.Greater(System.Math.Abs(dx), 0.01f, "Should have moved in X");
        }

        // ===== 模式切换 =====

        [Test]
        public void SetControlMode_Switches()
        {
            var (ctrl, _) = Create2RSetup();

            Assert.AreEqual(ControlMode.FK, ctrl.GetControlMode());

            ctrl.SetControlMode(ControlMode.IK);
            Assert.AreEqual(ControlMode.IK, ctrl.GetControlMode());

            ctrl.SetControlMode(ControlMode.FK);
            Assert.AreEqual(ControlMode.FK, ctrl.GetControlMode());
        }

        // ===== SyncToScene 验证 =====

        [Test]
        public void SyncToScene_AncestorDescendant_UpdatesLocalRotation()
        {
            var (ctrl, joints) = Create2RSetup();

            ctrl.SetJointAngle(0, Pi / 4f);

            // J0 的 LocalRotation 应该是 initRot * FromAxisAngle(z, π/4)
            // initRot = Identity, axis 默认 Forward (0,0,1)
            var expectedQuat = RMQuaternion.FromAxisAngle(RMVector3.Forward, Pi / 4f);
            var actualQuat = joints[0].LocalRotation;

            // 比较四元数（允许符号翻转）
            float dot = System.Math.Abs(
                actualQuat.X * expectedQuat.X +
                actualQuat.Y * expectedQuat.Y +
                actualQuat.Z * expectedQuat.Z +
                actualQuat.W * expectedQuat.W);
            Assert.AreEqual(1f, dot, 0.01f, "J0 rotation should match expected");
        }

        // ===== GetJointConfig =====

        [Test]
        public void GetJointConfig_ReturnsCorrect()
        {
            var (ctrl, _) = Create2RSetup();

            var cfg = ctrl.GetJointConfig(0);
            Assert.IsNotNull(cfg);
            Assert.AreEqual(JointType.Revolute, cfg.Type);

            Assert.IsNull(ctrl.GetJointConfig(-1));
            Assert.IsNull(ctrl.GetJointConfig(999));
        }

        // ===== 热更新 =====

        [Test]
        public void MarkDirty_ProcessDirtyFlag_Rebuilds()
        {
            var (ctrl, _) = Create2RSetup();

            ctrl.SetJointAngles(new float[] { 0.5f, 0.3f });
            var before = ctrl.GetEndEffectorPose().GetPosition();

            ctrl.MarkDirty();
            ctrl.ProcessDirtyFlag();

            // 处理后应该仍然能正常工作
            var after = ctrl.GetEndEffectorPose().GetPosition();
            Assert.AreEqual(before.X, after.X, Epsilon);
            Assert.AreEqual(before.Y, after.Y, Epsilon);
        }
    }
}
