using System.Runtime.CompilerServices;

namespace RobotMatrix.Math
{
    /// <summary>
    /// 数学工具类，纯 C# 实现，零 Unity 依赖。
    /// </summary>
    public static class RMMathUtils
    {
        public const float PI = (float)System.Math.PI;
        public const float TwoPI = PI * 2f;
        public const float HalfPI = PI * 0.5f;
        public const float Deg2Rad = PI / 180f;
        public const float Rad2Deg = 180f / PI;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Clamp(float value, float min, float max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static int Clamp(int value, int min, int max)
        {
            if (value < min) return min;
            if (value > max) return max;
            return value;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Abs(float value)
        {
            return value < 0f ? -value : value;
        }

        /// <summary>
        /// 两个浮点数是否近似相等。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool ApproximatelyEqual(float a, float b, float epsilon = 1e-6f)
        {
            return Abs(a - b) < epsilon;
        }

        /// <summary>
        /// 将角度归一化到 [-PI, PI] 范围。
        /// </summary>
        public static float NormalizeAngle(float angle)
        {
            while (angle > PI) angle -= TwoPI;
            while (angle < -PI) angle += TwoPI;
            return angle;
        }

        /// <summary>
        /// 将角度归一化到 [0, 2PI) 范围。
        /// </summary>
        public static float NormalizeAnglePositive(float angle)
        {
            while (angle >= TwoPI) angle -= TwoPI;
            while (angle < 0f) angle += TwoPI;
            return angle;
        }

        /// <summary>
        /// 计算旋转误差向量（轴角表示，3x1）。
        /// 用于 IK 求解器中计算当前旋转与目标旋转之间的差异。
        /// 
        /// 算法流程：
        ///   1. q_err = q_target * inverse(q_current)
        ///   2. 若 q_err.W 小于 0，取 -q_err（短路径选择，确保角度 <= PI）
        ///   3. sin_half = sqrt(q_err.X^2 + q_err.Y^2 + q_err.Z^2)
        ///   4. 若 sin_half 极小（< 1e-8）：返回 2 * (X, Y, Z)（泰勒近似）
        ///   5. half_angle = atan2(sin_half, q_err.W)
        ///   6. axis = (X, Y, Z) / sin_half
        ///   7. 返回 axis * (2 * half_angle)
        /// </summary>
        public static RMVector3 ComputeRotationError(RMQuaternion target, RMQuaternion current)
        {
            // 1. 计算误差四元数
            RMQuaternion qErr = target * RMQuaternion.Inverse(current);

            // 2. 短路径选择：确保 w >= 0，使旋转角度 <= PI
            if (qErr.W < 0f)
            {
                qErr.X = -qErr.X;
                qErr.Y = -qErr.Y;
                qErr.Z = -qErr.Z;
                qErr.W = -qErr.W;
            }

            // 3. 计算虚部大小
            float sinHalf = (float)System.Math.Sqrt(
                qErr.X * qErr.X + qErr.Y * qErr.Y + qErr.Z * qErr.Z
            );

            // 4. 小角度泰勒近似
            if (sinHalf < 1e-8f)
            {
                // 当 sin(half_angle) ≈ 0 时，angle ≈ 2 * sin(half_angle)
                // axis * angle ≈ 2 * (X, Y, Z)
                return new RMVector3(2f * qErr.X, 2f * qErr.Y, 2f * qErr.Z);
            }

            // 5. 正常情况
            float halfAngle = (float)System.Math.Atan2(sinHalf, qErr.W);
            float invSinHalf = 1f / sinHalf;
            float fullAngle = 2f * halfAngle;

            return new RMVector3(
                qErr.X * invSinHalf * fullAngle,
                qErr.Y * invSinHalf * fullAngle,
                qErr.Z * invSinHalf * fullAngle
            );
        }

        /// <summary>
        /// 检查浮点数是否为 NaN 或 Infinity。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        /// <summary>
        /// 检查向量各分量是否都是有限值。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool IsFinite(RMVector3 v)
        {
            return IsFinite(v.X) && IsFinite(v.Y) && IsFinite(v.Z);
        }

        /// <summary>
        /// 检查浮点数组中所有值是否有限。
        /// </summary>
        public static bool IsAllFinite(float[] values)
        {
            for (int i = 0; i < values.Length; i++)
                if (!IsFinite(values[i]))
                    return false;
            return true;
        }

        /// <summary>
        /// 线性插值。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Lerp(float a, float b, float t)
        {
            return a + (b - a) * t;
        }

        /// <summary>
        /// 返回两个值中较小的。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Min(float a, float b)
        {
            return a < b ? a : b;
        }

        /// <summary>
        /// 返回两个值中较大的。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Max(float a, float b)
        {
            return a > b ? a : b;
        }

        /// <summary>
        /// 浮点符号函数。
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float Sign(float value)
        {
            if (value > 0f) return 1f;
            if (value < 0f) return -1f;
            return 0f;
        }
    }
}
