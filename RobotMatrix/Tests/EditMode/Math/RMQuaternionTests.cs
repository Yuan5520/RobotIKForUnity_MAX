using NUnit.Framework;
using RobotMatrix.Math;

namespace RobotMatrix.Tests.Math
{
    [TestFixture]
    public class RMQuaternionTests
    {
        private const float Epsilon = 1e-5f;
        private const float Pi = (float)System.Math.PI;

        [Test]
        public void Identity_IsCorrect()
        {
            var q = RMQuaternion.Identity;
            Assert.AreEqual(0f, q.X, Epsilon);
            Assert.AreEqual(0f, q.Y, Epsilon);
            Assert.AreEqual(0f, q.Z, Epsilon);
            Assert.AreEqual(1f, q.W, Epsilon);
        }

        [Test]
        public void Multiply_IdentityLeftIsNoop()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 4f);
            var result = RMQuaternion.Identity * q;
            Assert.IsTrue(result == q);
        }

        [Test]
        public void Multiply_IdentityRightIsNoop()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 4f);
            var result = q * RMQuaternion.Identity;
            Assert.IsTrue(result == q);
        }

        [Test]
        public void Multiply_TimesInverse_ReturnsIdentity()
        {
            var q = RMQuaternion.FromAxisAngle(new RMVector3(1f, 1f, 1f).Normalized, 1.2f);
            var inv = RMQuaternion.Inverse(q);
            var result = q * inv;
            Assert.IsTrue(result == RMQuaternion.Identity);
        }

        [Test]
        public void Inverse_ReturnsCorrectResult()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 3f);
            var inv = RMQuaternion.Inverse(q);
            Assert.AreEqual(-q.X, inv.X, Epsilon);
            Assert.AreEqual(-q.Y, inv.Y, Epsilon);
            Assert.AreEqual(-q.Z, inv.Z, Epsilon);
            Assert.AreEqual(q.W, inv.W, Epsilon);
        }

        [Test]
        public void FromAxisAngle_ZeroAngle_ReturnsIdentity()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, 0f);
            Assert.IsTrue(q == RMQuaternion.Identity);
        }

        [Test]
        public void FromAxisAngle_90DegreesAroundY()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            float halfAngle = Pi / 4f;
            Assert.AreEqual(0f, q.X, Epsilon);
            Assert.AreEqual((float)System.Math.Sin(halfAngle), q.Y, Epsilon);
            Assert.AreEqual(0f, q.Z, Epsilon);
            Assert.AreEqual((float)System.Math.Cos(halfAngle), q.W, Epsilon);
        }

        [Test]
        public void ToAxisAngle_RecoversOriginal()
        {
            var axis = new RMVector3(1f, 2f, 3f).Normalized;
            float angle = 1.5f;
            var q = RMQuaternion.FromAxisAngle(axis, angle);

            RMQuaternion.ToAxisAngle(q, out RMVector3 outAxis, out float outAngle);
            Assert.AreEqual(angle, outAngle, Epsilon);
            Assert.AreEqual(axis.X, outAxis.X, Epsilon);
            Assert.AreEqual(axis.Y, outAxis.Y, Epsilon);
            Assert.AreEqual(axis.Z, outAxis.Z, Epsilon);
        }

        [Test]
        public void ToAxisAngle_Identity_ReturnsZeroAngle()
        {
            RMQuaternion.ToAxisAngle(RMQuaternion.Identity, out _, out float angle);
            Assert.AreEqual(0f, angle, Epsilon);
        }

        [Test]
        public void EulerRoundTrip_RecoversOriginal()
        {
            var euler = new RMVector3(0.3f, 0.5f, 0.7f);
            var q = RMQuaternion.EulerToQuaternion(euler);
            var recovered = RMQuaternion.QuaternionToEuler(q);
            Assert.AreEqual(euler.X, recovered.X, Epsilon);
            Assert.AreEqual(euler.Y, recovered.Y, Epsilon);
            Assert.AreEqual(euler.Z, recovered.Z, Epsilon);
        }

        [Test]
        public void EulerToQuaternion_ZeroEuler_ReturnsIdentity()
        {
            var q = RMQuaternion.EulerToQuaternion(RMVector3.Zero);
            Assert.IsTrue(q == RMQuaternion.Identity);
        }

        [Test]
        public void RotateVector_90DegreesAroundY_RotatesXToNegZ()
        {
            // 绕 Y 轴旋转 90 度：X -> -Z (右手坐标系)
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var result = RMQuaternion.RotateVector(q, RMVector3.Right);
            Assert.AreEqual(0f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(-1f, result.Z, Epsilon);
        }

        [Test]
        public void RotateVector_Identity_ReturnsOriginal()
        {
            var v = new RMVector3(1f, 2f, 3f);
            var result = RMQuaternion.RotateVector(RMQuaternion.Identity, v);
            Assert.AreEqual(v.X, result.X, Epsilon);
            Assert.AreEqual(v.Y, result.Y, Epsilon);
            Assert.AreEqual(v.Z, result.Z, Epsilon);
        }

        [Test]
        public void RotateVector_180DegreesAroundZ_FlipsXY()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Forward, Pi);
            var v = new RMVector3(1f, 0f, 0f);
            var result = RMQuaternion.RotateVector(q, v);
            Assert.AreEqual(-1f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(0f, result.Z, Epsilon);
        }

        [Test]
        public void Slerp_AtZero_ReturnsFirst()
        {
            var a = RMQuaternion.Identity;
            var b = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var result = RMQuaternion.Slerp(a, b, 0f);
            Assert.IsTrue(result == a);
        }

        [Test]
        public void Slerp_AtOne_ReturnsSecond()
        {
            var a = RMQuaternion.Identity;
            var b = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var result = RMQuaternion.Slerp(a, b, 1f);
            Assert.IsTrue(result == b);
        }

        [Test]
        public void Slerp_AtHalf_ReturnsMiddle()
        {
            var a = RMQuaternion.Identity;
            var b = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var mid = RMQuaternion.Slerp(a, b, 0.5f);

            // 中间点应该是绕 Y 轴旋转 45 度
            var expected = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 4f);
            float angle = RMQuaternion.Angle(mid, expected);
            Assert.AreEqual(0f, angle, 1e-4f);
        }

        [Test]
        public void Angle_SameQuaternion_ReturnsZero()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, 1f);
            Assert.AreEqual(0f, RMQuaternion.Angle(q, q), Epsilon);
        }

        [Test]
        public void Angle_90DegreesDifference()
        {
            var a = RMQuaternion.Identity;
            var b = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            Assert.AreEqual(Pi / 2f, RMQuaternion.Angle(a, b), 1e-4f);
        }

        [Test]
        public void Equality_SameRotation_DifferentSign()
        {
            var q = RMQuaternion.FromAxisAngle(RMVector3.Up, 1f);
            var negQ = new RMQuaternion(-q.X, -q.Y, -q.Z, -q.W);
            Assert.IsTrue(q == negQ);
        }

        [Test]
        public void Normalized_ReturnsUnitQuaternion()
        {
            var q = new RMQuaternion(1f, 2f, 3f, 4f);
            var n = q.Normalized;
            Assert.AreEqual(1f, n.Magnitude, Epsilon);
        }

        [Test]
        public void Conjugate_IsCorrect()
        {
            var q = new RMQuaternion(1f, 2f, 3f, 4f);
            var c = RMQuaternion.Conjugate(q);
            Assert.AreEqual(-1f, c.X, Epsilon);
            Assert.AreEqual(-2f, c.Y, Epsilon);
            Assert.AreEqual(-3f, c.Z, Epsilon);
            Assert.AreEqual(4f, c.W, Epsilon);
        }

        [Test]
        public void ChainedRotation_IsCorrect()
        {
            // 绕 X 旋转 90° 然后绕 Y 旋转 90°
            var rx = RMQuaternion.FromAxisAngle(RMVector3.Right, Pi / 2f);
            var ry = RMQuaternion.FromAxisAngle(RMVector3.Up, Pi / 2f);
            var combined = ry * rx; // 先 rx 再 ry

            // 将 Forward (0,0,1) 先绕 X 90°变成 (0,-1,0)，再绕 Y 90°变成 (0,-1,0)
            var v = RMVector3.Forward;
            var afterRx = RMQuaternion.RotateVector(rx, v);
            Assert.AreEqual(0f, afterRx.X, Epsilon);
            Assert.AreEqual(-1f, afterRx.Y, Epsilon);
            Assert.AreEqual(0f, afterRx.Z, Epsilon);

            var afterBoth = RMQuaternion.RotateVector(combined, v);
            Assert.AreEqual(0f, afterBoth.X, Epsilon);
            Assert.AreEqual(-1f, afterBoth.Y, Epsilon);
            Assert.AreEqual(0f, afterBoth.Z, Epsilon);
        }
    }
}
