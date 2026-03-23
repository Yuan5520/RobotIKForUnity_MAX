using System;
using System.Runtime.CompilerServices;

namespace RobotMatrix.Math
{
    /// <summary>
    /// 三维向量，纯 C# 实现，零 Unity 依赖。
    /// </summary>
    public struct RMVector3 : IEquatable<RMVector3>
    {
        public float X;
        public float Y;
        public float Z;

        public RMVector3(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        // ===== 静态常量 =====
        public static readonly RMVector3 Zero = new RMVector3(0f, 0f, 0f);
        public static readonly RMVector3 One = new RMVector3(1f, 1f, 1f);
        public static readonly RMVector3 Up = new RMVector3(0f, 1f, 0f);
        public static readonly RMVector3 Down = new RMVector3(0f, -1f, 0f);
        public static readonly RMVector3 Right = new RMVector3(1f, 0f, 0f);
        public static readonly RMVector3 Left = new RMVector3(-1f, 0f, 0f);
        public static readonly RMVector3 Forward = new RMVector3(0f, 0f, 1f);
        public static readonly RMVector3 Back = new RMVector3(0f, 0f, -1f);

        // ===== 属性 =====
        public float Magnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (float)System.Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public float SqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => X * X + Y * Y + Z * Z;
        }

        public RMVector3 Normalized
        {
            get
            {
                float mag = Magnitude;
                if (mag < 1e-10f)
                    return Zero;
                float inv = 1f / mag;
                return new RMVector3(X * inv, Y * inv, Z * inv);
            }
        }

        // ===== 静态方法 =====
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Dot(RMVector3 a, RMVector3 b)
        {
            return a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 Cross(RMVector3 a, RMVector3 b)
        {
            return new RMVector3(
                a.Y * b.Z - a.Z * b.Y,
                a.Z * b.X - a.X * b.Z,
                a.X * b.Y - a.Y * b.X
            );
        }

        public static RMVector3 Normalize(RMVector3 v)
        {
            return v.Normalized;
        }

        public static float Distance(RMVector3 a, RMVector3 b)
        {
            return (a - b).Magnitude;
        }

        public static RMVector3 Lerp(RMVector3 a, RMVector3 b, float t)
        {
            t = t < 0f ? 0f : (t > 1f ? 1f : t);
            return new RMVector3(
                a.X + (b.X - a.X) * t,
                a.Y + (b.Y - a.Y) * t,
                a.Z + (b.Z - a.Z) * t
            );
        }

        public static RMVector3 Project(RMVector3 v, RMVector3 onNormal)
        {
            float sqrMag = Dot(onNormal, onNormal);
            if (sqrMag < 1e-15f)
                return Zero;
            float dot = Dot(v, onNormal);
            return onNormal * (dot / sqrMag);
        }

        // ===== 运算符重载 =====
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator +(RMVector3 a, RMVector3 b)
        {
            return new RMVector3(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator -(RMVector3 a, RMVector3 b)
        {
            return new RMVector3(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator -(RMVector3 a)
        {
            return new RMVector3(-a.X, -a.Y, -a.Z);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator *(RMVector3 a, float d)
        {
            return new RMVector3(a.X * d, a.Y * d, a.Z * d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator *(float d, RMVector3 a)
        {
            return new RMVector3(a.X * d, a.Y * d, a.Z * d);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 operator /(RMVector3 a, float d)
        {
            float inv = 1f / d;
            return new RMVector3(a.X * inv, a.Y * inv, a.Z * inv);
        }

        public static bool operator ==(RMVector3 a, RMVector3 b)
        {
            return (a - b).SqrMagnitude < 1e-10f;
        }

        public static bool operator !=(RMVector3 a, RMVector3 b)
        {
            return !(a == b);
        }

        // ===== IEquatable =====
        public bool Equals(RMVector3 other)
        {
            return this == other;
        }

        public override bool Equals(object obj)
        {
            return obj is RMVector3 other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + X.GetHashCode();
                hash = hash * 31 + Y.GetHashCode();
                hash = hash * 31 + Z.GetHashCode();
                return hash;
            }
        }

        public override string ToString()
        {
            return $"({X:F4}, {Y:F4}, {Z:F4})";
        }
    }
}
