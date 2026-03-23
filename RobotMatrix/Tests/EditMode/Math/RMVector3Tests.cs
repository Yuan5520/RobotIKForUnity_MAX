using NUnit.Framework;
using RobotMatrix.Math;

namespace RobotMatrix.Tests.Math
{
    [TestFixture]
    public class RMVector3Tests
    {
        private const float Epsilon = 1e-6f;

        [Test]
        public void Constructor_SetsValues()
        {
            var v = new RMVector3(1f, 2f, 3f);
            Assert.AreEqual(1f, v.X, Epsilon);
            Assert.AreEqual(2f, v.Y, Epsilon);
            Assert.AreEqual(3f, v.Z, Epsilon);
        }

        [Test]
        public void StaticConstants_AreCorrect()
        {
            Assert.AreEqual(0f, RMVector3.Zero.X, Epsilon);
            Assert.AreEqual(0f, RMVector3.Zero.Y, Epsilon);
            Assert.AreEqual(0f, RMVector3.Zero.Z, Epsilon);

            Assert.AreEqual(0f, RMVector3.Up.X, Epsilon);
            Assert.AreEqual(1f, RMVector3.Up.Y, Epsilon);
            Assert.AreEqual(0f, RMVector3.Up.Z, Epsilon);

            Assert.AreEqual(1f, RMVector3.Right.X, Epsilon);
            Assert.AreEqual(0f, RMVector3.Forward.X, Epsilon);
            Assert.AreEqual(1f, RMVector3.Forward.Z, Epsilon);
        }

        [Test]
        public void Addition_ReturnsCorrectResult()
        {
            var a = new RMVector3(1f, 2f, 3f);
            var b = new RMVector3(4f, 5f, 6f);
            var result = a + b;
            Assert.AreEqual(5f, result.X, Epsilon);
            Assert.AreEqual(7f, result.Y, Epsilon);
            Assert.AreEqual(9f, result.Z, Epsilon);
        }

        [Test]
        public void Subtraction_ReturnsCorrectResult()
        {
            var a = new RMVector3(4f, 5f, 6f);
            var b = new RMVector3(1f, 2f, 3f);
            var result = a - b;
            Assert.AreEqual(3f, result.X, Epsilon);
            Assert.AreEqual(3f, result.Y, Epsilon);
            Assert.AreEqual(3f, result.Z, Epsilon);
        }

        [Test]
        public void Negation_ReturnsCorrectResult()
        {
            var v = new RMVector3(1f, -2f, 3f);
            var result = -v;
            Assert.AreEqual(-1f, result.X, Epsilon);
            Assert.AreEqual(2f, result.Y, Epsilon);
            Assert.AreEqual(-3f, result.Z, Epsilon);
        }

        [Test]
        public void ScalarMultiplication_ReturnsCorrectResult()
        {
            var v = new RMVector3(1f, 2f, 3f);
            var result1 = v * 2f;
            var result2 = 3f * v;
            Assert.AreEqual(2f, result1.X, Epsilon);
            Assert.AreEqual(4f, result1.Y, Epsilon);
            Assert.AreEqual(6f, result1.Z, Epsilon);
            Assert.AreEqual(3f, result2.X, Epsilon);
            Assert.AreEqual(6f, result2.Y, Epsilon);
            Assert.AreEqual(9f, result2.Z, Epsilon);
        }

        [Test]
        public void ScalarDivision_ReturnsCorrectResult()
        {
            var v = new RMVector3(2f, 4f, 6f);
            var result = v / 2f;
            Assert.AreEqual(1f, result.X, Epsilon);
            Assert.AreEqual(2f, result.Y, Epsilon);
            Assert.AreEqual(3f, result.Z, Epsilon);
        }

        [Test]
        public void Magnitude_ReturnsCorrectResult()
        {
            var v = new RMVector3(3f, 4f, 0f);
            Assert.AreEqual(5f, v.Magnitude, Epsilon);
        }

        [Test]
        public void SqrMagnitude_ReturnsCorrectResult()
        {
            var v = new RMVector3(1f, 2f, 3f);
            Assert.AreEqual(14f, v.SqrMagnitude, Epsilon);
        }

        [Test]
        public void Normalized_ReturnsUnitVector()
        {
            var v = new RMVector3(3f, 0f, 4f);
            var n = v.Normalized;
            Assert.AreEqual(1f, n.Magnitude, Epsilon);
            Assert.AreEqual(0.6f, n.X, Epsilon);
            Assert.AreEqual(0f, n.Y, Epsilon);
            Assert.AreEqual(0.8f, n.Z, Epsilon);
        }

        [Test]
        public void Normalized_ZeroVector_ReturnsZero()
        {
            var n = RMVector3.Zero.Normalized;
            Assert.AreEqual(0f, n.X, Epsilon);
            Assert.AreEqual(0f, n.Y, Epsilon);
            Assert.AreEqual(0f, n.Z, Epsilon);
        }

        [Test]
        public void Dot_ReturnsCorrectResult()
        {
            var a = new RMVector3(1f, 2f, 3f);
            var b = new RMVector3(4f, 5f, 6f);
            Assert.AreEqual(32f, RMVector3.Dot(a, b), Epsilon);
        }

        [Test]
        public void Dot_PerpendicularVectors_ReturnsZero()
        {
            Assert.AreEqual(0f, RMVector3.Dot(RMVector3.Up, RMVector3.Right), Epsilon);
            Assert.AreEqual(0f, RMVector3.Dot(RMVector3.Up, RMVector3.Forward), Epsilon);
        }

        [Test]
        public void Cross_ReturnsCorrectResult()
        {
            // X x Y = Z
            var result = RMVector3.Cross(RMVector3.Right, RMVector3.Up);
            Assert.AreEqual(0f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(1f, result.Z, Epsilon);
        }

        [Test]
        public void Cross_AntiCommutative()
        {
            var a = new RMVector3(1f, 2f, 3f);
            var b = new RMVector3(4f, 5f, 6f);
            var ab = RMVector3.Cross(a, b);
            var ba = RMVector3.Cross(b, a);
            Assert.AreEqual(-ab.X, ba.X, Epsilon);
            Assert.AreEqual(-ab.Y, ba.Y, Epsilon);
            Assert.AreEqual(-ab.Z, ba.Z, Epsilon);
        }

        [Test]
        public void Cross_ParallelVectors_ReturnsZero()
        {
            var a = new RMVector3(1f, 0f, 0f);
            var b = new RMVector3(3f, 0f, 0f);
            var result = RMVector3.Cross(a, b);
            Assert.AreEqual(0f, result.Magnitude, Epsilon);
        }

        [Test]
        public void Distance_ReturnsCorrectResult()
        {
            var a = new RMVector3(1f, 0f, 0f);
            var b = new RMVector3(4f, 0f, 0f);
            Assert.AreEqual(3f, RMVector3.Distance(a, b), Epsilon);
        }

        [Test]
        public void Lerp_ReturnsCorrectResult()
        {
            var a = new RMVector3(0f, 0f, 0f);
            var b = new RMVector3(10f, 20f, 30f);
            var mid = RMVector3.Lerp(a, b, 0.5f);
            Assert.AreEqual(5f, mid.X, Epsilon);
            Assert.AreEqual(10f, mid.Y, Epsilon);
            Assert.AreEqual(15f, mid.Z, Epsilon);
        }

        [Test]
        public void Lerp_ClampsParameter()
        {
            var a = new RMVector3(0f, 0f, 0f);
            var b = new RMVector3(10f, 0f, 0f);
            var below = RMVector3.Lerp(a, b, -1f);
            var above = RMVector3.Lerp(a, b, 2f);
            Assert.AreEqual(0f, below.X, Epsilon);
            Assert.AreEqual(10f, above.X, Epsilon);
        }

        [Test]
        public void Project_ReturnsCorrectResult()
        {
            var v = new RMVector3(3f, 4f, 0f);
            var onNormal = new RMVector3(1f, 0f, 0f);
            var result = RMVector3.Project(v, onNormal);
            Assert.AreEqual(3f, result.X, Epsilon);
            Assert.AreEqual(0f, result.Y, Epsilon);
            Assert.AreEqual(0f, result.Z, Epsilon);
        }

        [Test]
        public void Equality_SameVectors_ReturnsTrue()
        {
            var a = new RMVector3(1f, 2f, 3f);
            var b = new RMVector3(1f, 2f, 3f);
            Assert.IsTrue(a == b);
            Assert.IsFalse(a != b);
            Assert.IsTrue(a.Equals(b));
        }

        [Test]
        public void Equality_DifferentVectors_ReturnsFalse()
        {
            var a = new RMVector3(1f, 2f, 3f);
            var b = new RMVector3(1f, 2f, 4f);
            Assert.IsFalse(a == b);
            Assert.IsTrue(a != b);
        }

        [Test]
        public void ToString_FormatsCorrectly()
        {
            var v = new RMVector3(1f, 2f, 3f);
            string s = v.ToString();
            Assert.IsTrue(s.Contains("1.0000"));
            Assert.IsTrue(s.Contains("2.0000"));
            Assert.IsTrue(s.Contains("3.0000"));
        }
    }
}
