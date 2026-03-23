using NUnit.Framework;
using RobotMatrix.Math;

namespace RobotMatrix.Tests.Math
{
    [TestFixture]
    public class RMMatrixMNTests
    {
        private const float Epsilon = 1e-5f;

        [Test]
        public void Constructor_SetsRowsAndCols()
        {
            var m = new RMMatrixMN(3, 4);
            Assert.AreEqual(3, m.Rows);
            Assert.AreEqual(4, m.Cols);
        }

        [Test]
        public void Indexer_SetAndGet()
        {
            var m = new RMMatrixMN(2, 2);
            m[0, 0] = 1f;
            m[0, 1] = 2f;
            m[1, 0] = 3f;
            m[1, 1] = 4f;
            Assert.AreEqual(1f, m[0, 0], Epsilon);
            Assert.AreEqual(2f, m[0, 1], Epsilon);
            Assert.AreEqual(3f, m[1, 0], Epsilon);
            Assert.AreEqual(4f, m[1, 1], Epsilon);
        }

        [Test]
        public void Identity_IsCorrect()
        {
            var m = RMMatrixMN.Identity(3);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Assert.AreEqual(i == j ? 1f : 0f, m[i, j], Epsilon);
        }

        [Test]
        public void FromColumnVectors_Builds6xN()
        {
            var upper = new RMVector3[]
            {
                new RMVector3(1f, 2f, 3f),
                new RMVector3(4f, 5f, 6f)
            };
            var lower = new RMVector3[]
            {
                new RMVector3(7f, 8f, 9f),
                new RMVector3(10f, 11f, 12f)
            };

            var m = RMMatrixMN.FromColumnVectors(upper, lower);
            Assert.AreEqual(6, m.Rows);
            Assert.AreEqual(2, m.Cols);

            // 第一列
            Assert.AreEqual(1f, m[0, 0], Epsilon);
            Assert.AreEqual(2f, m[1, 0], Epsilon);
            Assert.AreEqual(3f, m[2, 0], Epsilon);
            Assert.AreEqual(7f, m[3, 0], Epsilon);
            Assert.AreEqual(8f, m[4, 0], Epsilon);
            Assert.AreEqual(9f, m[5, 0], Epsilon);

            // 第二列
            Assert.AreEqual(4f, m[0, 1], Epsilon);
            Assert.AreEqual(12f, m[5, 1], Epsilon);
        }

        // ===== 矩阵乘法 =====

        [Test]
        public void Multiply_IdentityTimesMatrix_ReturnsSame()
        {
            var a = new RMMatrixMN(2, 2);
            a[0, 0] = 1f; a[0, 1] = 2f;
            a[1, 0] = 3f; a[1, 1] = 4f;

            var id = RMMatrixMN.Identity(2);
            var result = RMMatrixMN.Multiply(id, a);

            Assert.AreEqual(1f, result[0, 0], Epsilon);
            Assert.AreEqual(2f, result[0, 1], Epsilon);
            Assert.AreEqual(3f, result[1, 0], Epsilon);
            Assert.AreEqual(4f, result[1, 1], Epsilon);
        }

        [Test]
        public void Multiply_2x3_Times_3x2_Returns2x2()
        {
            // A = [[1,2,3],[4,5,6]]  B = [[7,8],[9,10],[11,12]]
            var a = new RMMatrixMN(2, 3);
            a[0, 0] = 1f; a[0, 1] = 2f; a[0, 2] = 3f;
            a[1, 0] = 4f; a[1, 1] = 5f; a[1, 2] = 6f;

            var b = new RMMatrixMN(3, 2);
            b[0, 0] = 7f; b[0, 1] = 8f;
            b[1, 0] = 9f; b[1, 1] = 10f;
            b[2, 0] = 11f; b[2, 1] = 12f;

            var r = RMMatrixMN.Multiply(a, b);
            Assert.AreEqual(2, r.Rows);
            Assert.AreEqual(2, r.Cols);

            // r[0,0] = 1*7 + 2*9 + 3*11 = 7+18+33 = 58
            Assert.AreEqual(58f, r[0, 0], Epsilon);
            // r[0,1] = 1*8 + 2*10 + 3*12 = 8+20+36 = 64
            Assert.AreEqual(64f, r[0, 1], Epsilon);
            // r[1,0] = 4*7 + 5*9 + 6*11 = 28+45+66 = 139
            Assert.AreEqual(139f, r[1, 0], Epsilon);
            // r[1,1] = 4*8 + 5*10 + 6*12 = 32+50+72 = 154
            Assert.AreEqual(154f, r[1, 1], Epsilon);
        }

        // ===== 向量乘法 =====

        [Test]
        public void MultiplyVector_ReturnsCorrect()
        {
            var m = new RMMatrixMN(2, 3);
            m[0, 0] = 1f; m[0, 1] = 2f; m[0, 2] = 3f;
            m[1, 0] = 4f; m[1, 1] = 5f; m[1, 2] = 6f;

            var v = new float[] { 1f, 2f, 3f };
            var r = RMMatrixMN.MultiplyVector(m, v);

            Assert.AreEqual(2, r.Length);
            Assert.AreEqual(14f, r[0], Epsilon); // 1+4+9
            Assert.AreEqual(32f, r[1], Epsilon); // 4+10+18
        }

        // ===== 转置 =====

        [Test]
        public void Transpose_IsCorrect()
        {
            var m = new RMMatrixMN(2, 3);
            m[0, 0] = 1f; m[0, 1] = 2f; m[0, 2] = 3f;
            m[1, 0] = 4f; m[1, 1] = 5f; m[1, 2] = 6f;

            var t = RMMatrixMN.Transpose(m);
            Assert.AreEqual(3, t.Rows);
            Assert.AreEqual(2, t.Cols);
            Assert.AreEqual(1f, t[0, 0], Epsilon);
            Assert.AreEqual(4f, t[0, 1], Epsilon);
            Assert.AreEqual(2f, t[1, 0], Epsilon);
            Assert.AreEqual(5f, t[1, 1], Epsilon);
            Assert.AreEqual(3f, t[2, 0], Epsilon);
            Assert.AreEqual(6f, t[2, 1], Epsilon);
        }

        // ===== TransposeMultiply =====

        [Test]
        public void TransposeMultiply_EqualsExplicitTranspose()
        {
            var a = new RMMatrixMN(3, 2);
            a[0, 0] = 1f; a[0, 1] = 2f;
            a[1, 0] = 3f; a[1, 1] = 4f;
            a[2, 0] = 5f; a[2, 1] = 6f;

            var v = new float[] { 1f, 2f, 3f };

            // 显式转置后乘
            var at = RMMatrixMN.Transpose(a);
            var expected = RMMatrixMN.MultiplyVector(at, v);

            // 优化版本
            var actual = RMMatrixMN.TransposeMultiply(a, v);

            Assert.AreEqual(expected.Length, actual.Length);
            for (int i = 0; i < expected.Length; i++)
                Assert.AreEqual(expected[i], actual[i], Epsilon);
        }

        // ===== Add 和 ScalarMultiply =====

        [Test]
        public void Add_ReturnsCorrect()
        {
            var a = RMMatrixMN.Identity(2);
            var b = RMMatrixMN.Identity(2);
            var r = RMMatrixMN.Add(a, b);
            Assert.AreEqual(2f, r[0, 0], Epsilon);
            Assert.AreEqual(0f, r[0, 1], Epsilon);
            Assert.AreEqual(2f, r[1, 1], Epsilon);
        }

        [Test]
        public void ScalarMultiply_ReturnsCorrect()
        {
            var m = RMMatrixMN.Identity(2);
            var r = RMMatrixMN.ScalarMultiply(m, 3f);
            Assert.AreEqual(3f, r[0, 0], Epsilon);
            Assert.AreEqual(0f, r[0, 1], Epsilon);
            Assert.AreEqual(3f, r[1, 1], Epsilon);
        }

        // ===== Inverse6x6 =====

        [Test]
        public void Inverse6x6_Identity_ReturnsIdentity()
        {
            var id = RMMatrixMN.Identity(6);
            var inv = RMMatrixMN.Inverse6x6(id);
            Assert.IsNotNull(inv);

            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    Assert.AreEqual(i == j ? 1f : 0f, inv[i, j], Epsilon);
        }

        [Test]
        public void Inverse6x6_TimesOriginal_ReturnsIdentity()
        {
            // 构造一个已知的可逆 6x6 矩阵
            var m = RMMatrixMN.Identity(6);
            m[0, 0] = 2f; m[0, 1] = 1f;
            m[1, 0] = 1f; m[1, 1] = 3f; m[1, 2] = 0.5f;
            m[2, 2] = 4f; m[2, 3] = 1f;
            m[3, 3] = 2f; m[3, 4] = 0.5f;
            m[4, 4] = 3f; m[4, 5] = 1f;
            m[5, 5] = 5f;

            var inv = RMMatrixMN.Inverse6x6(m);
            Assert.IsNotNull(inv);

            var product = RMMatrixMN.Multiply(m, inv);
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    Assert.AreEqual(i == j ? 1f : 0f, product[i, j], 1e-4f,
                        $"product[{i},{j}] should be {(i == j ? 1 : 0)}");
        }

        [Test]
        public void Inverse6x6_KnownMatrix_Correct()
        {
            // 对角矩阵：逆就是对角元素取倒数
            var m = new RMMatrixMN(6, 6);
            for (int i = 0; i < 6; i++)
                m[i, i] = (float)(i + 1); // 1,2,3,4,5,6

            var inv = RMMatrixMN.Inverse6x6(m);
            Assert.IsNotNull(inv);

            for (int i = 0; i < 6; i++)
            {
                Assert.AreEqual(1f / (i + 1), inv[i, i], Epsilon);
                for (int j = 0; j < 6; j++)
                    if (i != j)
                        Assert.AreEqual(0f, inv[i, j], Epsilon);
            }
        }

        [Test]
        public void Inverse6x6_SingularMatrix_ReturnsNull()
        {
            // 全零矩阵是奇异的
            var m = new RMMatrixMN(6, 6);
            var inv = RMMatrixMN.Inverse6x6(m);
            Assert.IsNull(inv);
        }

        [Test]
        public void Inverse6x6_NearSingular_ReturnsNull()
        {
            // 两行相同 -> 奇异
            var m = RMMatrixMN.Identity(6);
            // 使第 0 行和第 1 行相同
            for (int j = 0; j < 6; j++)
                m[1, j] = m[0, j];
            var inv = RMMatrixMN.Inverse6x6(m);
            Assert.IsNull(inv);
        }

        // ===== CopyTo / Clear =====

        [Test]
        public void CopyTo_CreatesIdenticalCopy()
        {
            var m = new RMMatrixMN(2, 3);
            m[0, 0] = 1f; m[0, 2] = 5f; m[1, 1] = 3f;

            var target = new RMMatrixMN(2, 3);
            m.CopyTo(target);

            Assert.AreEqual(1f, target[0, 0], Epsilon);
            Assert.AreEqual(5f, target[0, 2], Epsilon);
            Assert.AreEqual(3f, target[1, 1], Epsilon);
        }

        [Test]
        public void Clear_ZerosAll()
        {
            var m = RMMatrixMN.Identity(3);
            m.Clear();
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    Assert.AreEqual(0f, m[i, j], Epsilon);
        }

        // ===== DLS 场景验证 =====

        [Test]
        public void DLS_Workflow_ProducesValidResult()
        {
            // 模拟 DLS 工作流：J(6x3), e(6x1), lambda=0.5
            int dof = 3;
            float lambda = 0.5f;

            // 构造简单的雅可比矩阵
            var jUpper = new RMVector3[]
            {
                new RMVector3(1f, 0f, 0f),
                new RMVector3(0f, 1f, 0f),
                new RMVector3(0f, 0f, 1f)
            };
            var jLower = new RMVector3[]
            {
                new RMVector3(0f, 0f, 1f),
                new RMVector3(0f, 0f, 0f),
                new RMVector3(1f, 0f, 0f)
            };

            var J = RMMatrixMN.FromColumnVectors(jUpper, jLower);
            Assert.AreEqual(6, J.Rows);
            Assert.AreEqual(3, J.Cols);

            // JJT = J * J^T (6x6)
            var JT = RMMatrixMN.Transpose(J);
            var JJT = RMMatrixMN.Multiply(J, JT);
            Assert.AreEqual(6, JJT.Rows);
            Assert.AreEqual(6, JJT.Cols);

            // JJT_damped = JJT + lambda^2 * I
            var lambdaI = RMMatrixMN.ScalarMultiply(RMMatrixMN.Identity(6), lambda * lambda);
            var JJT_damped = RMMatrixMN.Add(JJT, lambdaI);

            // 求逆
            var JJT_inv = RMMatrixMN.Inverse6x6(JJT_damped);
            Assert.IsNotNull(JJT_inv, "JJT_damped should be invertible");

            // 误差向量
            var e = new float[] { 0.1f, 0.2f, 0.3f, 0.01f, 0.02f, 0.03f };

            // temp = JJT_inv * e
            var temp = RMMatrixMN.MultiplyVector(JJT_inv, e);
            Assert.AreEqual(6, temp.Length);

            // delta_theta = J^T * temp
            var deltaTheta = RMMatrixMN.TransposeMultiply(J, temp);
            Assert.AreEqual(dof, deltaTheta.Length);

            // 确保结果是有限值
            for (int i = 0; i < dof; i++)
                Assert.IsTrue(RMMathUtils.IsFinite(deltaTheta[i]),
                    $"deltaTheta[{i}] should be finite, got {deltaTheta[i]}");
        }
    }
}
