using NUnit.Framework;
using RobotMatrix.Math;
using RobotMatrix.Kinematics;

namespace RobotMatrix.Tests
{
    public class JacobianCalculatorTests
    {
        private const float Epsilon = 1e-3f;
        private static readonly float Pi = RMMathUtils.PI;
        private JacobianCalculator _jacobian;
        private FKSolver _fk;

        [SetUp]
        public void SetUp()
        {
            _jacobian = new JacobianCalculator();
            _fk = new FKSolver();
        }

        // ===== 几何法 vs 数值微分法对比 =====

        /// <summary>
        /// 用数值微分法计算雅可比矩阵作为参考。
        /// 对每个关节施加微小扰动 δ，比较 FK 输出差异。
        /// </summary>
        private RMMatrixMN NumericalJacobian(RobotArmModel model, float[] jointAngles, float delta = 1e-5f)
        {
            int n = model.DOF;
            var J = new RMMatrixMN(6, n);

            // 基准 FK
            var baseEE = _fk.Solve(model, jointAngles);
            var basePos = baseEE.GetPosition();
            var baseRot = baseEE.GetRotation();

            for (int j = 0; j < n; j++)
            {
                // 扰动关节 j
                var perturbed = (float[])jointAngles.Clone();
                perturbed[j] += delta;

                var perturbedEE = _fk.Solve(model, perturbed);
                var perturbedPos = perturbedEE.GetPosition();
                var perturbedRot = perturbedEE.GetRotation();

                // 位置差分
                float invDelta = 1f / delta;
                J[0, j] = (perturbedPos.X - basePos.X) * invDelta;
                J[1, j] = (perturbedPos.Y - basePos.Y) * invDelta;
                J[2, j] = (perturbedPos.Z - basePos.Z) * invDelta;

                // 旋转差分（轴角）
                var rotError = RMMathUtils.ComputeRotationError(perturbedRot, baseRot);
                J[3, j] = rotError.X * invDelta;
                J[4, j] = rotError.Y * invDelta;
                J[5, j] = rotError.Z * invDelta;
            }

            return J;
        }

        private void AssertJacobiansClose(RMMatrixMN geometric, RMMatrixMN numerical, float eps)
        {
            Assert.AreEqual(geometric.Rows, numerical.Rows, "Row count mismatch");
            Assert.AreEqual(geometric.Cols, numerical.Cols, "Col count mismatch");

            for (int r = 0; r < geometric.Rows; r++)
                for (int c = 0; c < geometric.Cols; c++)
                    Assert.AreEqual(numerical[r, c], geometric[r, c], eps,
                        $"J[{r},{c}] mismatch: geo={geometric[r, c]:F6}, num={numerical[r, c]:F6}");
        }

        [Test]
        public void Jacobian_2R_AllZero_MatchesNumerical()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });
            var angles = new float[] { 0f, 0f };

            var geoJ = _jacobian.Compute(model, angles);
            var numJ = NumericalJacobian(model, angles);

            Assert.AreEqual(6, geoJ.Rows);
            Assert.AreEqual(2, geoJ.Cols);
            AssertJacobiansClose(geoJ, numJ, Epsilon);
        }

        [Test]
        public void Jacobian_2R_NonZeroAngles_MatchesNumerical()
        {
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(1f, 0f, 0f, 0f)
            });
            var angles = new float[] { Pi / 4f, Pi / 3f };

            var geoJ = _jacobian.Compute(model, angles);
            var numJ = NumericalJacobian(model, angles);

            AssertJacobiansClose(geoJ, numJ, Epsilon);
        }

        [Test]
        public void Jacobian_3R_WithAlpha_MatchesNumerical()
        {
            // 3-DOF 臂，有不同的 alpha
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(0f, Pi / 2f, 0.5f, 0f),
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(0.8f, -Pi / 2f, 0f, 0f)
            });
            var angles = new float[] { 0.3f, -0.5f, 0.7f };

            var geoJ = _jacobian.Compute(model, angles);
            var numJ = NumericalJacobian(model, angles);

            AssertJacobiansClose(geoJ, numJ, Epsilon);
        }

        [Test]
        public void Jacobian_6R_MatchesNumerical()
        {
            // 6-DOF 臂（类 Puma560 简化版）
            var model = RobotArmModel.FromDHParameters(new[]
            {
                new DHParameters(0f, Pi / 2f, 0.5f, 0f),
                new DHParameters(1f, 0f, 0f, 0f),
                new DHParameters(0f, Pi / 2f, 0f, 0f),
                new DHParameters(0f, -Pi / 2f, 1f, 0f),
                new DHParameters(0f, Pi / 2f, 0f, 0f),
                new DHParameters(0f, 0f, 0.2f, 0f)
            });
            var angles = new float[] { 0.1f, 0.2f, -0.3f, 0.4f, -0.5f, 0.6f };

            var geoJ = _jacobian.Compute(model, angles);
            var numJ = NumericalJacobian(model, angles);

            Assert.AreEqual(6, geoJ.Rows);
            Assert.AreEqual(6, geoJ.Cols);
            AssertJacobiansClose(geoJ, numJ, Epsilon);
        }

        [Test]
        public void Jacobian_WithPrismatic_MatchesNumerical()
        {
            // 混合关节：旋转 + 移动
            var model = RobotArmModel.FromDHParameters(
                new[]
                {
                    new DHParameters(0f, Pi / 2f, 0f, 0f),
                    new DHParameters(0f, 0f, 0f, 0f),  // 移动关节
                    new DHParameters(1f, 0f, 0f, 0f)
                },
                new[]
                {
                    JointType.Revolute,
                    JointType.Prismatic,
                    JointType.Revolute
                }
            );
            var angles = new float[] { 0.5f, 0.8f, -0.3f };

            var geoJ = _jacobian.Compute(model, angles);
            var numJ = NumericalJacobian(model, angles);

            AssertJacobiansClose(geoJ, numJ, Epsilon);
        }

        // ===== 维度验证 =====

        [Test]
        public void Jacobian_Dimensions_7DOF()
        {
            // 7-DOF 冗余臂
            var dh = new DHParameters[7];
            for (int i = 0; i < 7; i++)
                dh[i] = new DHParameters(0.5f, (i % 2 == 0) ? Pi / 2f : 0f, 0.1f, 0f);

            var model = RobotArmModel.FromDHParameters(dh);
            var angles = new float[7];

            var J = _jacobian.Compute(model, angles);

            Assert.AreEqual(6, J.Rows, "Rows should always be 6 (task space)");
            Assert.AreEqual(7, J.Cols, "Cols should be 7 (joint space)");
        }
    }
}
