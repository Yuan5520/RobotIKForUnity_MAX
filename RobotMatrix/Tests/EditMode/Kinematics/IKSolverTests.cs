using NUnit.Framework;
using RobotMatrix.Math;
using RobotMatrix.Kinematics;

namespace RobotMatrix.Tests
{
    public class IKSolverTests
    {
        private static readonly float Pi = RMMathUtils.PI;
        private IKSolver _ik;
        private FKSolver _fk;

        [SetUp]
        public void SetUp()
        {
            _ik = new IKSolver(new IKSolverConfig
            {
                MaxIterations = 100,
                PositionThreshold = 0.001f,
                RotationThreshold = 0.01f,
                DampingFactor = 0.1f
            });
            _fk = new FKSolver();
        }

        // ===== 辅助方法 =====

        private RobotArmModel Create2RPlanar(float a1 = 1f, float a2 = 1f)
        {
            return RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(a1, 0f, 0f, 0f),
                new DHParameters(a2, 0f, 0f, 0f)
            });
        }

        private RobotArmModel Create6DOF()
        {
            return RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(0f, Pi / 2f, 0.5f, 0f),
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(0f, Pi / 2f, 0f, 0f),
                new DHParameters(0f, -Pi / 2f, 1f, 0f),
                new DHParameters(0f, Pi / 2f, 0f, 0f),
                new DHParameters(0f, 0f, 0.2f, 0f)
            });
        }

        /// <summary>
        /// FK→IK→FK 环回验证：先用 FK 算出目标位姿，再用 IK 求解，
        /// 然后用 FK 验证结果是否回到目标。
        /// </summary>
        private void AssertFKIKRoundTrip(RobotArmModel model, float[] targetAngles,
                                          float posEps = 0.01f, float rotEps = 0.05f)
        {
            // FK 计算目标位姿
            var targetPose = _fk.Solve(model, targetAngles);
            var targetPos = targetPose.GetPosition();
            var targetRot = targetPose.GetRotation();

            // 从零位出发求解 IK
            var initAngles = new float[model.DOF];
            var result = _ik.Solve(model, initAngles, targetPos, targetRot);

            // 用求解出的角度做 FK 验证
            var verifyPose = _fk.Solve(model, result.JointAngles);
            var verifyPos = verifyPose.GetPosition();
            var verifyRot = verifyPose.GetRotation();

            // 位置误差
            float pErr = (verifyPos - targetPos).Magnitude;
            Assert.Less(pErr, posEps,
                $"Position error {pErr:F6} exceeds threshold {posEps}");

            // 旋转误差
            var rErrVec = RMMathUtils.ComputeRotationError(targetRot, verifyRot);
            float rErr = rErrVec.Magnitude;
            Assert.Less(rErr, rotEps,
                $"Rotation error {rErr:F6} exceeds threshold {rotEps}");
        }

        // ===== 2R 平面臂测试 =====

        [Test]
        public void IK_2R_ReachableTarget_Converges()
        {
            var model = Create2RPlanar();
            AssertFKIKRoundTrip(model, new float[] { Pi / 4f, Pi / 3f });
        }

        [Test]
        public void IK_2R_StraightArm_Converges()
        {
            var model = Create2RPlanar();
            AssertFKIKRoundTrip(model, new float[] { 0f, 0f }, 0.01f, 0.1f);
        }

        [Test]
        public void IK_2R_FoldedArm_Converges()
        {
            var model = Create2RPlanar();
            // θ1=0, θ2=π/2 → 末端在 (1, 1, 0)
            AssertFKIKRoundTrip(model, new float[] { 0f, Pi / 2f });
        }

        // ===== 不可达目标 =====

        [Test]
        public void IK_2R_Unreachable_NotConverged()
        {
            var model = Create2RPlanar(1f, 1f);
            // 工作空间半径 = 2, 目标在 (5, 0, 0) → 不可达
            var result = _ik.Solve(model,
                new float[] { 0f, 0f },
                new RMVector3(5f, 0f, 0f),
                RMQuaternion.Identity);

            Assert.IsFalse(result.Converged, "Should not converge for unreachable target");
            Assert.Greater(result.PositionError, 0.001f, "Position error should be significant");
        }

        // ===== 关节限位 =====

        [Test]
        public void IK_2R_WithLimits_Respected()
        {
            var model = Create2RPlanar();
            // 限制两个关节在 [-π/4, π/4]
            for (int i = 0; i < 2; i++)
            {
                model.JointConfigs[i].HasLimits = true;
                model.JointConfigs[i].MinLimit = -Pi / 4f;
                model.JointConfigs[i].MaxLimit = Pi / 4f;
            }

            // 目标需要大角度才能达到，限位应被尊重
            var targetPose = _fk.Solve(model, new float[] { Pi / 4f, Pi / 4f });
            var result = _ik.Solve(model,
                new float[] { 0f, 0f },
                targetPose.GetPosition(),
                targetPose.GetRotation());

            for (int i = 0; i < 2; i++)
            {
                Assert.GreaterOrEqual(result.JointAngles[i], -Pi / 4f - 1e-5f,
                    $"Joint {i} below min limit");
                Assert.LessOrEqual(result.JointAngles[i], Pi / 4f + 1e-5f,
                    $"Joint {i} above max limit");
            }
        }

        // ===== 6-DOF 臂 =====

        [Test]
        public void IK_6DOF_ReachableTarget_Converges()
        {
            var model = Create6DOF();
            AssertFKIKRoundTrip(model,
                new float[] { 0.3f, -0.4f, 0.2f, 0.5f, -0.3f, 0.1f },
                0.01f, 0.05f);
        }

        [Test]
        public void IK_6DOF_SmallMove_ConvergesQuickly()
        {
            var model = Create6DOF();
            var startAngles = new float[] { 0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f };

            // 从 startAngles 出发，目标是稍微偏移的位姿
            var targetAngles = new float[] { 0.15f, 0.12f, 0.08f, 0.11f, 0.09f, 0.13f };
            var targetPose = _fk.Solve(model, targetAngles);

            var result = _ik.Solve(model, startAngles,
                targetPose.GetPosition(),
                targetPose.GetRotation());

            Assert.IsTrue(result.Converged, "Should converge for small move");
            Assert.Less(result.IterationsUsed, 30, "Small move should converge quickly");
        }

        // ===== NaN 守卫 =====

        [Test]
        public void IK_NaN_Guard_DoesNotCrash()
        {
            var model = Create2RPlanar();
            // 给一个极端目标，不应崩溃
            var result = _ik.Solve(model,
                new float[] { 0f, 0f },
                new RMVector3(100f, 100f, 100f),
                RMQuaternion.Identity);

            Assert.IsNotNull(result);
            Assert.IsNotNull(result.JointAngles);
            // 结果角度不应包含 NaN
            for (int i = 0; i < result.JointAngles.Length; i++)
                Assert.IsFalse(float.IsNaN(result.JointAngles[i]),
                    $"Joint {i} is NaN");
        }

        // ===== IKResult 信息完整性 =====

        [Test]
        public void IKResult_ContainsAllInfo()
        {
            var model = Create2RPlanar();
            var targetPose = _fk.Solve(model, new float[] { Pi / 4f, Pi / 4f });

            var result = _ik.Solve(model,
                new float[] { 0f, 0f },
                targetPose.GetPosition(),
                targetPose.GetRotation());

            Assert.IsNotNull(result.JointAngles);
            Assert.AreEqual(2, result.JointAngles.Length);
            Assert.GreaterOrEqual(result.IterationsUsed, 0);
            Assert.GreaterOrEqual(result.PositionError, 0f);
            Assert.GreaterOrEqual(result.RotationError, 0f);
        }
    }
}
