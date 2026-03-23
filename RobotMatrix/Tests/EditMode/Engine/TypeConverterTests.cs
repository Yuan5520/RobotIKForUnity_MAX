using NUnit.Framework;
using UnityEngine;
using RobotMatrix.Math;
using IEngine;
using IEngine.Unity;

namespace RobotMatrix.Tests
{
    public class TypeConverterTests
    {
        private const float Epsilon = 1e-6f;

        // ===== Vector3 互转 =====

        [Test]
        public void Vector3_ToRM_Roundtrip()
        {
            var uv = new Vector3(1.5f, -2.3f, 4.7f);
            var rm = TypeConverter.ToRM(uv);
            var back = TypeConverter.ToUnity(rm);

            Assert.AreEqual(uv.x, back.x, Epsilon);
            Assert.AreEqual(uv.y, back.y, Epsilon);
            Assert.AreEqual(uv.z, back.z, Epsilon);
        }

        [Test]
        public void Vector3_ToUnity_Roundtrip()
        {
            var rm = new RMVector3(3.14f, -1.0f, 0.001f);
            var uv = TypeConverter.ToUnity(rm);
            var back = TypeConverter.ToRM(uv);

            Assert.AreEqual(rm.X, back.X, Epsilon);
            Assert.AreEqual(rm.Y, back.Y, Epsilon);
            Assert.AreEqual(rm.Z, back.Z, Epsilon);
        }

        [Test]
        public void Vector3_Zero()
        {
            var uv = Vector3.zero;
            var rm = TypeConverter.ToRM(uv);

            Assert.AreEqual(0f, rm.X, Epsilon);
            Assert.AreEqual(0f, rm.Y, Epsilon);
            Assert.AreEqual(0f, rm.Z, Epsilon);
        }

        // ===== Quaternion 互转 =====

        [Test]
        public void Quaternion_ToRM_Roundtrip()
        {
            var uq = Quaternion.Euler(30f, 45f, 60f);
            var rm = TypeConverter.ToRM(uq);
            var back = TypeConverter.ToUnity(rm);

            Assert.AreEqual(uq.x, back.x, Epsilon);
            Assert.AreEqual(uq.y, back.y, Epsilon);
            Assert.AreEqual(uq.z, back.z, Epsilon);
            Assert.AreEqual(uq.w, back.w, Epsilon);
        }

        [Test]
        public void Quaternion_Identity()
        {
            var uq = Quaternion.identity;
            var rm = TypeConverter.ToRM(uq);

            Assert.AreEqual(0f, rm.X, Epsilon);
            Assert.AreEqual(0f, rm.Y, Epsilon);
            Assert.AreEqual(0f, rm.Z, Epsilon);
            Assert.AreEqual(1f, rm.W, Epsilon);
        }

        [Test]
        public void Quaternion_ToUnity_Roundtrip()
        {
            // 绕 Y 轴旋转 90 度
            var rm = RMQuaternion.FromAxisAngle(RMVector3.Up, RMMathUtils.PI / 2f);
            var uq = TypeConverter.ToUnity(rm);
            var back = TypeConverter.ToRM(uq);

            Assert.AreEqual(rm.X, back.X, Epsilon);
            Assert.AreEqual(rm.Y, back.Y, Epsilon);
            Assert.AreEqual(rm.Z, back.Z, Epsilon);
            Assert.AreEqual(rm.W, back.W, Epsilon);
        }

        // ===== Matrix4x4 互转 =====

        [Test]
        public void Matrix4x4_Identity_Roundtrip()
        {
            var um = UnityEngine.Matrix4x4.identity;
            var rm = TypeConverter.ToRM(um);
            var back = TypeConverter.ToUnity(rm);

            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    Assert.AreEqual(um[row, col], back[row, col], Epsilon,
                        $"Mismatch at [{row},{col}]");
        }

        [Test]
        public void Matrix4x4_TRS_Roundtrip()
        {
            var pos = new Vector3(1f, 2f, 3f);
            var rot = Quaternion.Euler(10f, 20f, 30f);
            var scale = Vector3.one;
            var um = UnityEngine.Matrix4x4.TRS(pos, rot, scale);

            var rm = TypeConverter.ToRM(um);
            var back = TypeConverter.ToUnity(rm);

            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    Assert.AreEqual(um[row, col], back[row, col], Epsilon,
                        $"Mismatch at [{row},{col}]");
        }

        [Test]
        public void Matrix4x4_Position_Preserved()
        {
            var pos = new Vector3(5f, -3f, 7f);
            var um = UnityEngine.Matrix4x4.Translate(pos);
            var rm = TypeConverter.ToRM(um);

            // RMMatrix4x4 位置在第 4 列 (0-indexed: [0,3], [1,3], [2,3])
            var rmPos = rm.GetPosition();
            Assert.AreEqual(pos.x, rmPos.X, Epsilon);
            Assert.AreEqual(pos.y, rmPos.Y, Epsilon);
            Assert.AreEqual(pos.z, rmPos.Z, Epsilon);
        }

        // ===== Color 互转 =====

        [Test]
        public void Color_ToRM_Roundtrip()
        {
            var uc = new Color(0.2f, 0.4f, 0.6f, 0.8f);
            var rm = TypeConverter.ToRM(uc);
            var back = TypeConverter.ToUnity(rm);

            Assert.AreEqual(uc.r, back.r, Epsilon);
            Assert.AreEqual(uc.g, back.g, Epsilon);
            Assert.AreEqual(uc.b, back.b, Epsilon);
            Assert.AreEqual(uc.a, back.a, Epsilon);
        }

        [Test]
        public void Color_Predefined()
        {
            var rmRed = RMColor.Red;
            var uRed = TypeConverter.ToUnity(rmRed);

            Assert.AreEqual(1f, uRed.r, Epsilon);
            Assert.AreEqual(0f, uRed.g, Epsilon);
            Assert.AreEqual(0f, uRed.b, Epsilon);
            Assert.AreEqual(1f, uRed.a, Epsilon);
        }
    }
}
