using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 类型与 RM 类型之间的双向转换工具。
    /// </summary>
    public static class TypeConverter
    {
        // ===== Vector3 =====

        public static RMVector3 ToRM(Vector3 v)
        {
            return new RMVector3(v.x, v.y, v.z);
        }

        public static Vector3 ToUnity(RMVector3 v)
        {
            return new Vector3(v.X, v.Y, v.Z);
        }

        // ===== Quaternion =====

        public static RMQuaternion ToRM(Quaternion q)
        {
            return new RMQuaternion(q.x, q.y, q.z, q.w);
        }

        public static Quaternion ToUnity(RMQuaternion q)
        {
            return new Quaternion(q.X, q.Y, q.Z, q.W);
        }

        // ===== Matrix4x4 =====

        public static RMMatrix4x4 ToRM(Matrix4x4 m)
        {
            var rm = new RMMatrix4x4();
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    rm[row, col] = m[row, col];
            return rm;
        }

        public static Matrix4x4 ToUnity(RMMatrix4x4 rm)
        {
            var m = new Matrix4x4();
            for (int row = 0; row < 4; row++)
                for (int col = 0; col < 4; col++)
                    m[row, col] = rm[row, col];
            return m;
        }

        // ===== Color =====

        public static RMColor ToRM(Color c)
        {
            return new RMColor(c.r, c.g, c.b, c.a);
        }

        public static Color ToUnity(RMColor c)
        {
            return new Color(c.R, c.G, c.B, c.A);
        }
    }
}
