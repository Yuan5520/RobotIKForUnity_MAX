using NUnit.Framework;
using RobotMatrix.Math;

namespace RobotMatrix.Tests.Math
{
    [TestFixture]
    public class RMMathUtilsTests
    {
        private const float Epsilon = 1e-5f;
        private const float Pi = RMMathUtils.PI;

        // ===== 基础工具方法 =====

        [Test]
        public void Clamp_WithinRange_ReturnsValue()
        {
            Assert.AreEqual(5f, RMMathUtils.Clamp(5f, 0f, 10f), Epsilon);
        }

        [Test]
        public void Clamp_BelowMin_ReturnsMin()
        {
            Assert.AreEqual(0f, RMMathUtils.Clamp(-1f, 0f, 10f), Epsilon);
        }

        [Test]
        public void Clamp_AboveMax_ReturnsMax()
        {
            Assert.AreEqual(10f, RMMathUtils.Clamp(15f, 0f, 10f), Epsilon);
        }

        [Test]
        public void Deg2Rad_AndBack()
        {
            float deg = 90f;
            float rad = deg * RMMathUtils.Deg2Rad;
            Assert.AreEqual(Pi / 2f, rad, Epsilon);
            float back = rad * RMMathUtils.Rad2Deg;
            Assert.AreEqual(90f, back, Epsilon);
        }

        [Test]
        public void ApproximatelyEqual_CloseValues_ReturnsTrue()
        {
            Assert.IsTrue(RMMathUtils.ApproximatelyEqual(1f, 1f + 1e-7f));
        }

        [Test]
        public void ApproximatelyEqual_FarValues_ReturnsFalse()
        {
            Assert.IsFalse(RMMathUtils.ApproximatelyEqual(1f, 2f));
        }

        [Test]
        public void NormalizeAngle_LargePositive()
        {
            float result = RMMathUtils.NormalizeAngle(3f * Pi);
            Assert.AreEqual(Pi, RMMathUtils.Abs(result), Epsilon);
        }

        [Test]
        public void NormalizeAngle_LargeNegative()
        {
            float result = RMMathUtils.NormalizeAngle(-3f * Pi);
            Assert.AreEqual(Pi, RMMathUtils.Abs(result), Epsilon);
        }

        [Test]
        public void NormalizeAngle_WithinRange_Unchanged()
        {
            float result = RMMathUtils.NormalizeAngle(1f);
            Assert.AreEqual(1f, result, Epsilon);
        }

        [Test]
        public void IsFinite_Normal_ReturnsTrue()
        {
            Assert.IsTrue(RMMathUtils.IsFinite(1f));
            Assert.IsTrue(RMMathUtils.IsFinite(0f));
            Assert.IsTrue(RMMathUtils.IsFinite(-999f));
        }

        [Test]
        public void IsFinite_NaN_ReturnsFalse()
        {
            Assert.IsFalse(RMMathUtils.IsFinite(float.NaN));
        }

        [Test]
        public void IsFinite_Infinity_ReturnsFalse()
        {
            Assert.IsFalse(RMMathUtils.IsFinite(float.PositiveInfinity));
            Assert.IsFalse(RMMathUtils.IsFinite(float.NegativeInfinity));
        }

        [Test]
        public void IsAllFinite_AllValid_ReturnsTrue()
        {
            Assert.IsTrue(RMMathUtils.IsAllFinite(new float[] { 1f, 2f, 3f }));
        }

        [Test]
        public void IsAllFinite_ContainsNaN_ReturnsFalse()
        {
            Assert.IsFalse(RMMathUtils.IsAllFinite(new float[] { 1f, float.NaN, 3f }));
        }

        // ===== ComputeRotationError =====

        [Test]
        public void ComputeRotationError_SameRotation_ReturnsZero()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, 1.0f);
            var error = RMMathUtils.ComputeRotationError(q, q);
            Assert.AreEqual(0f, error.Magnitude, Epsilon);
        }

        [Test]
        public void ComputeRotationError_Identity_ReturnsZero()
        {
            var error = RMMathUtils.ComputeRotationError(
                RMQuaternion.Identity, RMQuaternion.Identity);
            Assert.AreEqual(0f, error.Magnitude, Epsilon);
        }

        [Test]
        public void ComputeRotationError_90DegAroundY_ReturnsAxisTimesHalfPi()
        {
            var target = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var current = RMQuaternion.Identity;
            var error = RMMathUtils.ComputeRotationError(target, current);

            // 误差应为 Y 轴方向 * PI/2
            Assert.AreEqual(0f, error.X, Epsilon);
            Assert.AreEqual(Pi / 2f, error.Y, 1e-4f);
            Assert.AreEqual(0f, error.Z, Epsilon);
        }

        [Test]
        public void ComputeRotationError_90DegAroundX_ReturnsCorrect()
        {
            var target = RMQuaternion.FromAxisAngle(RMVector3.Right, Pi / 2f);
            var current = RMQuaternion.Identity;
            var error = RMMathUtils.ComputeRotationError(target, current);

            Assert.AreEqual(Pi / 2f, error.X, 1e-4f);
            Assert.AreEqual(0f, error.Y, Epsilon);
            Assert.AreEqual(0f, error.Z, Epsilon);
        }

        [Test]
        public void ComputeRotationError_179Deg_DoesNotFlipTo181()
        {
            // 绕 Z 轴旋转 179°
            float angle = 179f * RMMathUtils.Deg2Rad;
            var target = RMQuaternion.FromAxisAngle(RMVector3.Forward, angle);
            var current = RMQuaternion.Identity;
            var error = RMMathUtils.ComputeRotationError(target, current);

            // 误差大小应接近 179° (3.1241 rad)，不应翻转到 181° (3.1590 rad)
            float errorMag = error.Magnitude;
            Assert.AreEqual(angle, errorMag, 0.01f);

            // 方向应为 Z 轴
            var errorNorm = error.Normalized;
            Assert.AreEqual(0f, errorNorm.X, 0.01f);
            Assert.AreEqual(0f, errorNorm.Y, 0.01f);
            Assert.AreEqual(1f, RMMathUtils.Abs(errorNorm.Z), 0.01f);
        }

        [Test]
        public void ComputeRotationError_VerySmallAngle_HighPrecision()
        {
            // 绕 Y 轴旋转 0.001°
            float angle = 0.001f * RMMathUtils.Deg2Rad;
            var target = RMQuaternion.FromAxisAngle(RMVector3.Up, angle);
            var current = RMQuaternion.Identity;
            var error = RMMathUtils.ComputeRotationError(target, current);

            // 误差大小应接近 0.001°
            Assert.AreEqual(angle, error.Magnitude, 1e-6f);

            // 方向应为 Y 轴（使用泰勒近似路径）
            Assert.AreEqual(0f, error.X, 1e-6f);
            Assert.AreEqual(0f, error.Z, 1e-6f);
        }

        [Test]
        public void ComputeRotationError_NegativeWQuaternion_ShortPath()
        {
            // 构造 W < 0 的四元数（与 W > 0 表示同一旋转）
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 4f);
            var negQ = new RMQuaternion(-q.X, -q.Y, -q.Z, -q.W);

            var error = RMMathUtils.ComputeRotationError(negQ, RMQuaternion.Identity);

            // 应该走短路径，误差大小为 PI/4
            Assert.AreEqual(Pi / 4f, error.Magnitude, 1e-4f);
        }

        [Test]
        public void ComputeRotationError_ArbitraryAxis_ReturnsCorrect()
        {
            var axis = new RMVector3(1f, 1f, 1f).Normalized;
            float angle = 1.0f; // 约 57°
            var target = RMQuaternion.FromAxisAngle(axis, angle);
            var current = RMQuaternion.Identity;
            var error = RMMathUtils.ComputeRotationError(target, current);

            // 误差大小应等于旋转角度
            Assert.AreEqual(angle, error.Magnitude, 1e-4f);

            // 误差方向应与旋转轴一致
            var errorNorm = error.Normalized;
            Assert.AreEqual(axis.X, errorNorm.X, 0.01f);
            Assert.AreEqual(axis.Y, errorNorm.Y, 0.01f);
            Assert.AreEqual(axis.Z, errorNorm.Z, 0.01f);
        }

        [Test]
        public void ComputeRotationError_NonIdentityCurrent()
        {
            // current 不是 identity
            var current = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 4f);
            var target = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var error = RMMathUtils.ComputeRotationError(target, current);

            // 差异应为绕 Y 轴 PI/4
            Assert.AreEqual(0f, error.X, Epsilon);
            Assert.AreEqual(Pi / 4f, error.Y, 1e-4f);
            Assert.AreEqual(0f, error.Z, Epsilon);
        }

        // ===== 其他工具方法 =====

        [Test]
        public void Min_ReturnsSmaller()
        {
            Assert.AreEqual(1f, RMMathUtils.Min(1f, 2f), Epsilon);
            Assert.AreEqual(-5f, RMMathUtils.Min(-5f, 3f), Epsilon);
        }

        [Test]
        public void Max_ReturnsLarger()
        {
            Assert.AreEqual(2f, RMMathUtils.Max(1f, 2f), Epsilon);
            Assert.AreEqual(3f, RMMathUtils.Max(-5f, 3f), Epsilon);
        }

        [Test]
        public void Sign_ReturnsCorrect()
        {
            Assert.AreEqual(1f, RMMathUtils.Sign(5f), Epsilon);
            Assert.AreEqual(-1f, RMMathUtils.Sign(-3f), Epsilon);
            Assert.AreEqual(0f, RMMathUtils.Sign(0f), Epsilon);
        }

        [Test]
        public void Lerp_ReturnsCorrect()
        {
            Assert.AreEqual(5f, RMMathUtils.Lerp(0f, 10f, 0.5f), Epsilon);
            Assert.AreEqual(0f, RMMathUtils.Lerp(0f, 10f, 0f), Epsilon);
            Assert.AreEqual(10f, RMMathUtils.Lerp(0f, 10f, 1f), Epsilon);
        }
    }
}
