using System;
using System.Runtime.CompilerServices;

namespace RobotMatrix.Math
{
    /// <summary>
    /// 四元数，纯 C# 实现，零 Unity 依赖。
    /// 存储格式: (X, Y, Z, W)，其中 W 为实部。
    /// </summary>
    public struct RMQuaternion : IEquatable<RMQuaternion>
    {
        public float X;
        public float Y;
        public float Z;
        public float W;

        public RMQuaternion(float x, float y, float z, float w)
        {
            X = x;
            Y = y;
            Z = z;
            W = w;
        }

        // ===== 静态常量 =====
        public static readonly RMQuaternion Identity = new RMQuaternion(0f, 0f, 0f, 1f);

        // ===== 属性 =====
        public float Magnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => (float)System.Math.Sqrt(X * X + Y * Y + Z * Z + W * W);
        }

        public float SqrMagnitude
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get => X * X + Y * Y + Z * Z + W * W;
        }

        public RMQuaternion Normalized
        {
            get
            {
                float mag = Magnitude;
                if (mag < 1e-10f)
                    return Identity;
                float inv = 1f / mag;
                return new RMQuaternion(X * inv, Y * inv, Z * inv, W * inv);
            }
        }

        // ===== 四元数乘法 =====
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMQuaternion operator *(RMQuaternion a, RMQuaternion b)
        {
            return new RMQuaternion(
                a.W * b.X + a.X * b.W + a.Y * b.Z - a.Z * b.Y,
                a.W * b.Y - a.X * b.Z + a.Y * b.W + a.Z * b.X,
                a.W * b.Z + a.X * b.Y - a.Y * b.X + a.Z * b.W,
                a.W * b.W - a.X * b.X - a.Y * b.Y - a.Z * b.Z
            );
        }

        // ===== 逆四元数 =====
        public static RMQuaternion Inverse(RMQuaternion q)
        {
            float sqrMag = q.SqrMagnitude;
            if (sqrMag < 1e-15f)
                return Identity;
            float inv = 1f / sqrMag;
            return new RMQuaternion(-q.X * inv, -q.Y * inv, -q.Z * inv, q.W * inv);
        }

        // ===== 共轭 =====
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMQuaternion Conjugate(RMQuaternion q)
        {
            return new RMQuaternion(-q.X, -q.Y, -q.Z, q.W);
        }

        // ===== 轴角转换 =====

        /// <summary>
        /// 从轴角创建四元数。axis 应为单位向量，angle 为弧度。
        /// </summary>
        public static RMQuaternion FromAxisAngle(RMVector3 axis, float angle)
        {
            float halfAngle = angle * 0.5f;
            float sinHalf = (float)System.Math.Sin(halfAngle);
            float cosHalf = (float)System.Math.Cos(halfAngle);
            return new RMQuaternion(
                axis.X * sinHalf,
                axis.Y * sinHalf,
                axis.Z * sinHalf,
                cosHalf
            );
        }

        /// <summary>
        /// 提取轴角表示。identity 四元数返回 (Forward, 0)。
        /// </summary>
        public static void ToAxisAngle(RMQuaternion q, out RMVector3 axis, out float angle)
        {
            // 确保单位四元数
            RMQuaternion qn = q.Normalized;

            // 确保 w >= 0（短路径）
            if (qn.W < 0f)
            {
                qn.X = -qn.X;
                qn.Y = -qn.Y;
                qn.Z = -qn.Z;
                qn.W = -qn.W;
            }

            float sinHalfSq = qn.X * qn.X + qn.Y * qn.Y + qn.Z * qn.Z;

            if (sinHalfSq < 1e-14f)
            {
                // 几乎无旋转
                axis = RMVector3.Forward;
                angle = 0f;
                return;
            }

            float sinHalf = (float)System.Math.Sqrt(sinHalfSq);
            float halfAngle = (float)System.Math.Atan2(sinHalf, qn.W);
            angle = halfAngle * 2f;

            float invSinHalf = 1f / sinHalf;
            axis = new RMVector3(qn.X * invSinHalf, qn.Y * invSinHalf, qn.Z * invSinHalf);
        }

        // ===== 欧拉角转换 =====

        /// <summary>
        /// 从欧拉角创建四元数。euler 为 (X, Y, Z) 弧度，旋转顺序 ZYX（先绕 Z，再 Y，最后 X）。
        /// </summary>
        public static RMQuaternion EulerToQuaternion(RMVector3 euler)
        {
            float cx = (float)System.Math.Cos(euler.X * 0.5f);
            float sx = (float)System.Math.Sin(euler.X * 0.5f);
            float cy = (float)System.Math.Cos(euler.Y * 0.5f);
            float sy = (float)System.Math.Sin(euler.Y * 0.5f);
            float cz = (float)System.Math.Cos(euler.Z * 0.5f);
            float sz = (float)System.Math.Sin(euler.Z * 0.5f);

            return new RMQuaternion(
                sx * cy * cz - cx * sy * sz,
                cx * sy * cz + sx * cy * sz,
                cx * cy * sz - sx * sy * cz,
                cx * cy * cz + sx * sy * sz
            );
        }

        /// <summary>
        /// 四元数转欧拉角（弧度），返回 (X, Y, Z)，旋转顺序 ZYX。
        /// </summary>
        public static RMVector3 QuaternionToEuler(RMQuaternion q)
        {
            // Roll (X)
            float sinr_cosp = 2f * (q.W * q.X + q.Y * q.Z);
            float cosr_cosp = 1f - 2f * (q.X * q.X + q.Y * q.Y);
            float roll = (float)System.Math.Atan2(sinr_cosp, cosr_cosp);

            // Pitch (Y)
            float sinp = 2f * (q.W * q.Y - q.Z * q.X);
            float pitch;
            if (System.Math.Abs(sinp) >= 1f)
                pitch = (float)CopySign(System.Math.PI / 2.0, sinp);
            else
                pitch = (float)System.Math.Asin(sinp);

            // Yaw (Z)
            float siny_cosp = 2f * (q.W * q.Z + q.X * q.Y);
            float cosy_cosp = 1f - 2f * (q.Y * q.Y + q.Z * q.Z);
            float yaw = (float)System.Math.Atan2(siny_cosp, cosy_cosp);

            return new RMVector3(roll, pitch, yaw);
        }

        // ===== Slerp =====
        public static RMQuaternion Slerp(RMQuaternion a, RMQuaternion b, float t)
        {
            t = t < 0f ? 0f : (t > 1f ? 1f : t);

            float dot = a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;

            // 确保短路径
            if (dot < 0f)
            {
                b = new RMQuaternion(-b.X, -b.Y, -b.Z, -b.W);
                dot = -dot;
            }

            if (dot > 0.9995f)
            {
                // 非常接近，线性插值
                var result = new RMQuaternion(
                    a.X + (b.X - a.X) * t,
                    a.Y + (b.Y - a.Y) * t,
                    a.Z + (b.Z - a.Z) * t,
                    a.W + (b.W - a.W) * t
                );
                return result.Normalized;
            }

            float theta0 = (float)System.Math.Acos(dot);
            float theta = theta0 * t;
            float sinTheta = (float)System.Math.Sin(theta);
            float sinTheta0 = (float)System.Math.Sin(theta0);

            float s0 = (float)System.Math.Cos(theta) - dot * sinTheta / sinTheta0;
            float s1 = sinTheta / sinTheta0;

            return new RMQuaternion(
                s0 * a.X + s1 * b.X,
                s0 * a.Y + s1 * b.Y,
                s0 * a.Z + s1 * b.Z,
                s0 * a.W + s1 * b.W
            ).Normalized;
        }

        // ===== 向量旋转 =====

        /// <summary>
        /// 使用四元数旋转一个向量: v' = q * v * q^-1
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMVector3 RotateVector(RMQuaternion q, RMVector3 v)
        {
            // 优化版本，避免两次四元数乘法
            float tx = 2f * (q.Y * v.Z - q.Z * v.Y);
            float ty = 2f * (q.Z * v.X - q.X * v.Z);
            float tz = 2f * (q.X * v.Y - q.Y * v.X);

            return new RMVector3(
                v.X + q.W * tx + (q.Y * tz - q.Z * ty),
                v.Y + q.W * ty + (q.Z * tx - q.X * tz),
                v.Z + q.W * tz + (q.X * ty - q.Y * tx)
            );
        }

        /// <summary>
        /// 计算两个四元数之间的角度（弧度）。
        /// </summary>
        public static float Angle(RMQuaternion a, RMQuaternion b)
        {
            float dot = System.Math.Abs(a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W);
            if (dot > 1f) dot = 1f;
            return 2f * (float)System.Math.Acos(dot);
        }

        // ===== IEquatable =====
        public static bool operator ==(RMQuaternion a, RMQuaternion b)
        {
            // 四元数 q 和 -q 表示相同旋转
            float dot = a.X * b.X + a.Y * b.Y + a.Z * b.Z + a.W * b.W;
            return System.Math.Abs(System.Math.Abs(dot) - 1f) < 1e-6f;
        }

        public static bool operator !=(RMQuaternion a, RMQuaternion b)
        {
            return !(a == b);
        }

        public bool Equals(RMQuaternion other)
        {
            return this == other;
        }

        public override bool Equals(object obj)
        {
            return obj is RMQuaternion other && Equals(other);
        }

        public override int GetHashCode()
        {
            // 由于 q == -q，规范化为 W >= 0
            float sign = W >= 0 ? 1f : -1f;
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + (X * sign).GetHashCode();
                hash = hash * 31 + (Y * sign).GetHashCode();
                hash = hash * 31 + (Z * sign).GetHashCode();
                hash = hash * 31 + (W * sign).GetHashCode();
                return hash;
            }
        }

        public override string ToString()
        {
            return $"({X:F4}, {Y:F4}, {Z:F4}, {W:F4})";
        }

        // ===== 内部工具 =====
        private static double CopySign(double magnitude, double sign)
        {
            return sign >= 0 ? System.Math.Abs(magnitude) : -System.Math.Abs(magnitude);
        }
    }
}
