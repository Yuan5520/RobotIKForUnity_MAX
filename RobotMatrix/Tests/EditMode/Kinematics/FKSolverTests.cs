using NUnit.Framework;
using RobotMatrix.Math;
using RobotMatrix.Kinematics;

namespace RobotMatrix.Tests
{
    public class FKSolverTests
    {
        private const float Epsilon = 1e-5f;
        private static readonly float Pi = RMMathUtils.PI;
        private FKSolver _solver;

        [SetUp]
        public void SetUp()
        {
            _solver = new FKSolver();
        }

        // ===== 2R 平面臂 =====

        private RobotArmModel Create2RPlanar(float a1 = 1f, float a2 = 1f)
        {
            return RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(a1, 0f, 0f, 0f),
                new DHParameters(a2, 0f, 0f, 0f)
            });
        }

        [Test]
        public void FK_2R_AllZero_EndAtSumOfLinks()
        {
            // θ1=0, θ2=0 → 末端在 (a1+a2, 0, 0)
            var model = Create2RPlanar(1f, 1f);
            var ee = _solver.Solve(model, new float[] { 0f, 0f });
            var pos = ee.GetPosition();

            Assert.AreEqual(2f, pos.X, Epsilon, "X");
            Assert.AreEqual(0f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(0f, pos.Z, Epsilon, "Z");
        }

        [Test]
        public void FK_2R_Theta1_90_EndRotated()
        {
            // θ1=π/2, θ2=0 → 末端在 (0, 2, 0)
            var model = Create2RPlanar(1f, 1f);
            var ee = _solver.Solve(model, new float[] { Pi / 2f, 0f });
            var pos = ee.GetPosition();

            Assert.AreEqual(0f, pos.X, Epsilon, "X");
            Assert.AreEqual(2f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(0f, pos.Z, Epsilon, "Z");
        }

        [Test]
        public void FK_2R_Theta1_90_Theta2_90()
        {
            // θ1=π/2, θ2=π/2 → 末端在 (-1, 1, 0)
            var model = Create2RPlanar(1f, 1f);
            var ee = _solver.Solve(model, new float[] { Pi / 2f, Pi / 2f });
            var pos = ee.GetPosition();

            Assert.AreEqual(-1f, pos.X, Epsilon, "X");
            Assert.AreEqual(1f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(0f, pos.Z, Epsilon, "Z");
        }

        [Test]
        public void FK_2R_Theta2_180_Folded()
        {
            // θ1=0, θ2=π → 末端在 (0, 0, 0)，臂完全折叠
            var model = Create2RPlanar(1f, 1f);
            var ee = _solver.Solve(model, new float[] { 0f, Pi });
            var pos = ee.GetPosition();

            Assert.AreEqual(0f, pos.X, Epsilon, "X");
            Assert.AreEqual(0f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(0f, pos.Z, Epsilon, "Z");
        }

        [Test]
        public void FK_2R_AsymmetricLengths()
        {
            // a1=2, a2=3, θ1=0, θ2=0 → 末端在 (5, 0, 0)
            var model = Create2RPlanar(2f, 3f);
            var ee = _solver.Solve(model, new float[] { 0f, 0f });
            var pos = ee.GetPosition();

            Assert.AreEqual(5f, pos.X, Epsilon, "X");
        }

        // ===== 3D: 带 alpha 的臂 =====

        [Test]
        public void FK_WithAlpha90_RotatesPlane()
        {
            // J1: a=0, α=π/2, d=1, θ=0 → 提升 d=1 并旋转后续平面
            // J2: a=1, α=0, d=0, θ=0 → 在旋转后的平面延伸 a=1
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(0f, Pi / 2f, 1f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });

            var ee = _solver.Solve(model, new float[] { 0f, 0f });
            var pos = ee.GetPosition();

            // T1: a=0, α=π/2, d=1, θ=0
            // | 1   0    0   0 |
            // | 0   0   -1   0 |
            // | 0   1    0   1 |
            // | 0   0    0   1 |
            // T2: a=1, α=0, d=0, θ=0
            // | 1  -1  0  1 |   →  | 1  0  0  1 |
            // T = T1*T2, position column = T1 * (1,0,0,1)^T
            // x = 1*1 + 0*0 + 0*0 + 0*1 = 1
            // y = 0*1 + 0*0 + (-1)*0 + 0*1 = 0
            // z = 0*1 + 1*0 + 0*0 + 1*1 = 1
            Assert.AreEqual(1f, pos.X, Epsilon, "X");
            Assert.AreEqual(0f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(1f, pos.Z, Epsilon, "Z");
        }

        // ===== 移动关节 =====

        [Test]
        public void FK_PrismaticJoint_TranslatesAlongZ()
        {
            // 单个移动关节：a=0, α=0, d=0, θ_off=0
            // 移动关节的 displacement 加到 d 上
            var model = RobotArmModel.FromDHParameters(
                new[] { new DHParameters(0f, 0f, 0f, 0f) },
                new[] { JointType.Prismatic }
            );

            // displacement = 3 → d = 0 + 3 = 3
            var ee = _solver.Solve(model, new float[] { 3f });
            var pos = ee.GetPosition();

            Assert.AreEqual(0f, pos.X, Epsilon, "X");
            Assert.AreEqual(0f, pos.Y, Epsilon, "Y");
            Assert.AreEqual(3f, pos.Z, Epsilon, "Z");
        }

        // ===== JointStates 更新 =====

        [Test]
        public void FK_UpdatesJointStates()
        {
            var model = Create2RPlanar(1f, 1f);
            _solver.Solve(model, new float[] { 0f, 0f });

            // J0 的 WorldTransform 位置应为 (1, 0, 0)
            var j0Pos = model.JointStates[0].WorldTransform.GetPosition();
            Assert.AreEqual(1f, j0Pos.X, Epsilon, "J0.X");
            Assert.AreEqual(0f, j0Pos.Y, Epsilon, "J0.Y");

            // J1 的 WorldTransform 位置应为 (2, 0, 0)
            var j1Pos = model.JointStates[1].WorldTransform.GetPosition();
            Assert.AreEqual(2f, j1Pos.X, Epsilon, "J1.X");
            Assert.AreEqual(0f, j1Pos.Y, Epsilon, "J1.Y");
        }

        // ===== EndEffectorOffset =====

        [Test]
        public void FK_WithEndEffectorOffset()
        {
            var model = Create2RPlanar(1f, 1f);
            // 末端偏移: 沿 x 方向 0.5
            model.EndEffectorOffset = RMMatrix4x4.FromTranslation(new RMVector3(0.5f, 0f, 0f));

            var ee = _solver.Solve(model, new float[] { 0f, 0f });
            var pos = ee.GetPosition();

            Assert.AreEqual(2.5f, pos.X, Epsilon, "X with offset");
        }

        // ===== ThetaOffset =====

        [Test]
        public void FK_ThetaOffset_AppliedCorrectly()
        {
            // J1: θ_offset=π/2 → 即使 jointAngle=0，也已旋转 90°
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, Pi / 2f)
            });

            var ee = _solver.Solve(model, new float[] { 0f });
            var pos = ee.GetPosition();

            // θ = π/2 + 0 = π/2 → 末端在 (0, 1, 0)
            Assert.AreEqual(0f, pos.X, Epsilon, "X");
            Assert.AreEqual(1f, pos.Y, Epsilon, "Y");
        }
    }
}
