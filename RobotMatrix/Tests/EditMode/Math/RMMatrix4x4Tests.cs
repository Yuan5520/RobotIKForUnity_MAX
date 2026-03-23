using NUnit.Framework;
using RobotMatrix.Math;

namespace RobotMatrix.Tests.Math
{
    [TestFixture]
    public class RMMatrix4x4Tests
    {
        private const float Epsilon = 1e-5f;
        private const float Pi = (float)System.Math.PI;

        [Test]
        public void Identity_IsCorrect()
        {
            var m = RMMatrix4x4.Identity;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    Assert.AreEqual(r == c ? 1f : 0f, m[r, c], Epsilon);
        }

        [Test]
        public void Multiply_ByIdentity_ReturnsSelf()
        {
            var m = RMMatrix4x4.FromTranslation(new RMVector3(1f, 2f, 3f));
            var result = RMMatrix4x4.Multiply(m, RMMatrix4x4.Identity);
            Assert.IsTrue(result.Equals(m));
        }

        [Test]
        public void Multiply_TwoTranslations_AddsThem()
        {
            var t1 = RMMatrix4x4.FromTranslation(new RMVector3(1f, 0f, 0f));
            var t2 = RMMatrix4x4.FromTranslation(new RMVector3(0f, 2f, 0f));
            var result = t1 * t2;
            var pos = result.GetPosition();
            Assert.AreEqual(1f, pos.X, Epsilon);
            Assert.AreEqual(2f, pos.Y, Epsilon);
            Assert.AreEqual(0f, pos.Z, Epsilon);
        }

        [Test]
        public void Inverse_TimesOriginal_ReturnsIdentity()
        {
            var m = RMMatrix4x4.FromTranslationRotation(
                new RMVector3(3f, -1f, 7f),
                RMQuaternion.FromAxisAngle(new RMVector3(1f, 1f, 0f).Normalized, 1.2f)
            );
            var inv = RMMatrix4x4.Inverse(m);
            var result = m * inv;

            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    Assert.AreEqual(r == c ? 1f : 0f, result[r, c], 1e-4f);
        }

        [Test]
        public void Transpose_IsCorrect()
        {
            var m = RMMatrix4x4.Identity;
            m[0, 1] = 5f;
            m[1, 0] = 7f;
            var t = RMMatrix4x4.Transpose(m);
            Assert.AreEqual(7f, t[0, 1], Epsilon);
            Assert.AreEqual(5f, t[1, 0], Epsilon);
        }

        [Test]
        public void FromTranslationRotation_GetPositionAndRotation_RoundTrip()
        {
            var pos = new RMVector3(1f, 2f, 3f);
            var rot = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 3f);
            var m = RMMatrix4x4.FromTranslationRotation(pos, rot);

            var outPos = m.GetPosition();
            Assert.AreEqual(pos.X, outPos.X, Epsilon);
            Assert.AreEqual(pos.Y, outPos.Y, Epsilon);
            Assert.AreEqual(pos.Z, outPos.Z, Epsilon);

            var outRot = m.GetRotation();
            float angle = RMQuaternion.Angle(rot, outRot);
            Assert.AreEqual(0f, angle, 1e-4f);
        }

        [Test]
        public void GetColumn_ReturnsCorrect()
        {
            var pos = new RMVector3(10f, 20f, 30f);
            var m = RMMatrix4x4.FromTranslation(pos);
            var col3 = m.GetColumn(3);
            Assert.AreEqual(10f, col3.X, Epsilon);
            Assert.AreEqual(20f, col3.Y, Epsilon);
            Assert.AreEqual(30f, col3.Z, Epsilon);
        }

        [Test]
        public void MultiplyPoint_TranslatesPoint()
        {
            var m = RMMatrix4x4.FromTranslation(new RMVector3(5f, 0f, 0f));
            var result = m.MultiplyPoint(new RMVector3(1f, 2f, 3f));
            Assert.AreEqual(6f, result.X, Epsilon);
            Assert.AreEqual(2f, result.Y, Epsilon);
            Assert.AreEqual(3f, result.Z, Epsilon);
        }

        [Test]
        public void MultiplyDirection_DoesNotTranslate()
        {
            var m = RMMatrix4x4.FromTranslation(new RMVector3(5f, 0f, 0f));
            var result = m.MultiplyDirection(new RMVector3(1f, 0f, 0f));
            Assert.AreEqual(1f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(0f, result.Z, Epsilon);
        }

        [Test]
        public void RotationAroundAxis_Y90_RotatesXToNegZ()
        {
            var m = RMMatrix4x4.RotationAroundAxis(RMVector3.Up, Pi / 2f);
            var result = m.MultiplyDirection(RMVector3.Right);
            Assert.AreEqual(0f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(-1f, result.Z, Epsilon);
        }

        // ===== DHMatrix 手算验证 =====

        [Test]
        public void DHMatrix_AllZeroParams_ReturnsIdentity()
        {
            var m = RMMatrix4x4.DHMatrix(0f, 0f, 0f, 0f);
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    Assert.AreEqual(r == c ? 1f : 0f, m[r, c], Epsilon,
                        $"DHMatrix(0,0,0,0)[{r},{c}] should be {(r == c ? 1 : 0)}");
        }

        [Test]
        public void DHMatrix_PureTranslation_d()
        {
            // a=0, alpha=0, d=5, theta=0 -> 沿 z 平移 5
            var m = RMMatrix4x4.DHMatrix(0f, 0f, 5f, 0f);
            var pos = m.GetPosition();
            Assert.AreEqual(0f, pos.X, Epsilon);
            Assert.AreEqual(0f, pos.Y, Epsilon);
            Assert.AreEqual(5f, pos.Z, Epsilon);
        }

        [Test]
        public void DHMatrix_PureTranslation_a()
        {
            // a=3, alpha=0, d=0, theta=0 -> 沿 x 平移 3
            var m = RMMatrix4x4.DHMatrix(3f, 0f, 0f, 0f);
            var pos = m.GetPosition();
            Assert.AreEqual(3f, pos.X, Epsilon);
            Assert.AreEqual(0f, pos.Y, Epsilon);
            Assert.AreEqual(0f, pos.Z, Epsilon);
        }

        [Test]
        public void DHMatrix_PureRotation_theta90()
        {
            // a=0, alpha=0, d=0, theta=90° -> 绕 z 轴旋转 90°
            var m = RMMatrix4x4.DHMatrix(0f, 0f, 0f, Pi / 2f);
            // cos(90)=0, sin(90)=1
            Assert.AreEqual(0f, m[0, 0], Epsilon);  // cos(theta)
            Assert.AreEqual(1f, m[1, 0], Epsilon);  // sin(theta)
            Assert.AreEqual(-1f, m[0, 1], Epsilon); // -sin(theta)*cos(alpha)
            Assert.AreEqual(0f, m[1, 1], Epsilon);  // cos(theta)*cos(alpha)
        }

        [Test]
        public void DHMatrix_KnownDH_2RLink()
        {
            // 标准 2R 平面机械臂第一关节:
            // a=1, alpha=0, d=0, theta=0 -> 沿 x 平移 1
            var t1 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, 0f);
            // 第二关节: a=1, alpha=0, d=0, theta=0
            var t2 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, 0f);

            var result = t1 * t2;
            var endPos = result.GetPosition();

            // 两个连杆长度各1，零角度 -> 末端在 (2, 0, 0)
            Assert.AreEqual(2f, endPos.X, Epsilon);
            Assert.AreEqual(0f, endPos.Y, Epsilon);
            Assert.AreEqual(0f, endPos.Z, Epsilon);
        }

        [Test]
        public void DHMatrix_2RLink_WithAngles()
        {
            // 2R 平面: L1=1, L2=1, theta1=90°, theta2=0°
            // 末端应在 (0, 1+1, 0) = (0, 2, 0)... 不对
            // DH: 关节1 theta=90° -> x' 旋转 90°，然后沿 x' 平移 a=1 -> (0,1,0)
            // 关节2 theta=0° -> 不旋转，沿 x'' 平移 a=1 -> x'' 方向同 x'（绕 z 旋转了 90°所以 x' = y）
            // 最终末端 (0, 2, 0)
            var t1 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, Pi / 2f);
            var t2 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, 0f);
            var result = t1 * t2;
            var endPos = result.GetPosition();
            Assert.AreEqual(0f, endPos.X, Epsilon);
            Assert.AreEqual(2f, endPos.Y, Epsilon);
            Assert.AreEqual(0f, endPos.Z, Epsilon);
        }

        [Test]
        public void DHMatrix_2RLink_BothRotated()
        {
            // 2R 平面: L1=1, L2=1, theta1=45°, theta2=45°
            float t1Angle = Pi / 4f;
            float t2Angle = Pi / 4f;
            var m1 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, t1Angle);
            var m2 = RMMatrix4x4.DHMatrix(1f, 0f, 0f, t2Angle);
            var result = m1 * m2;
            var endPos = result.GetPosition();

            // 手算：
            // 关节1后: x1 = cos(45)*1 = 0.7071, y1 = sin(45)*1 = 0.7071
            // 关节2的坐标系绕z旋转了45°，再转45°=90°
            // x2相对=cos(90)*1=0, y2相对=sin(90)*1=1 -> 世界: 旋转90° -> (-sin90, cos90)*1 不对
            // 直接用矩阵计算验证:
            float c1 = (float)System.Math.Cos(t1Angle);
            float s1 = (float)System.Math.Sin(t1Angle);
            float c12 = (float)System.Math.Cos(t1Angle + t2Angle);
            float s12 = (float)System.Math.Sin(t1Angle + t2Angle);
            float expectedX = 1f * c1 + 1f * c12;
            float expectedY = 1f * s1 + 1f * s12;

            Assert.AreEqual(expectedX, endPos.X, Epsilon);
            Assert.AreEqual(expectedY, endPos.Y, Epsilon);
            Assert.AreEqual(0f, endPos.Z, Epsilon);
        }

        [Test]
        public void DHMatrix_WithAlpha90_RotatesZAxis()
        {
            // a=0, alpha=90°, d=0, theta=0
            // 这应该绕 x 轴旋转 z 轴 90°
            var m = RMMatrix4x4.DHMatrix(0f, Pi / 2f, 0f, 0f);
            // DH 公式 col2: m[0,2]=st*sa=0, m[1,2]=-ct*sa=-1, m[2,2]=ca=0
            Assert.AreEqual(0f, m[0, 2], Epsilon);
            Assert.AreEqual(-1f, m[1, 2], Epsilon);
            Assert.AreEqual(0f, m[2, 2], Epsilon);
        }

        [Test]
        public void OperatorMultiply_SameAsMultiply()
        {
            var a = RMMatrix4x4.FromTranslation(new RMVector3(1f, 0f, 0f));
            var b = RMMatrix4x4.RotationAroundAxis(RMVector3.Up, 0.5f);
            var r1 = RMMatrix4x4.Multiply(a, b);
            var r2 = a * b;
            Assert.IsTrue(r1.Equals(r2));
        }
    }
}
