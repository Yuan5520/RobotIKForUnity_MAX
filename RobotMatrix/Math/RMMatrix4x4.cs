using System;
using System.Runtime.CompilerServices;

namespace RobotMatrix.Math
{
    /// <summary>
    /// 4x4 齐次变换矩阵，纯 C# 实现，零 Unity 依赖。
    /// 列优先存储（与 OpenGL/Unity 一致）：m[col * 4 + row]
    /// </summary>
    public struct RMMatrix4x4 : IEquatable<RMMatrix4x4>
    {
        // 列优先存储：m00=第0列第0行, m10=第0列第1行 ...
        // 列 0          列 1          列 2          列 3
        public float m00, m10, m20, m30;
        public float m01, m11, m21, m31;
        public float m02, m12, m22, m32;
        public float m03, m13, m23, m33;

        // ===== 索引器 =====
        public float this[int row, int col]
        {
            get
            {
                // 列优先展开
                int index = col * 4 + row;
                switch (index)
                {
                    case 0: return m00; case 1: return m10; case 2: return m20; case 3: return m30;
                    case 4: return m01; case 5: return m11; case 6: return m21; case 7: return m31;
                    case 8: return m02; case 9: return m12; case 10: return m22; case 11: return m32;
                    case 12: return m03; case 13: return m13; case 14: return m23; case 15: return m33;
                    default: throw new IndexOutOfRangeException();
                }
            }
            set
            {
                int index = col * 4 + row;
                switch (index)
                {
                    case 0: m00 = value; break; case 1: m10 = value; break; case 2: m20 = value; break; case 3: m30 = value; break;
                    case 4: m01 = value; break; case 5: m11 = value; break; case 6: m21 = value; break; case 7: m31 = value; break;
                    case 8: m02 = value; break; case 9: m12 = value; break; case 10: m22 = value; break; case 11: m32 = value; break;
                    case 12: m03 = value; break; case 13: m13 = value; break; case 14: m23 = value; break; case 15: m33 = value; break;
                    default: throw new IndexOutOfRangeException();
                }
            }
        }

        // ===== 静态常量 =====
        public static readonly RMMatrix4x4 Identity = new RMMatrix4x4
        {
            m00 = 1f, m11 = 1f, m22 = 1f, m33 = 1f
        };

        public static readonly RMMatrix4x4 Zero = new RMMatrix4x4();

        // ===== 位置/旋转提取 =====

        /// <summary>
        /// 获取变换矩阵的平移部分（第3列前3个元素）。
        /// </summary>
        public RMVector3 GetPosition()
        {
            return new RMVector3(m03, m13, m23);
        }

        /// <summary>
        /// 设置变换矩阵的平移部分。
        /// </summary>
        public void SetPosition(RMVector3 pos)
        {
            m03 = pos.X;
            m13 = pos.Y;
            m23 = pos.Z;
        }

        /// <summary>
        /// 获取列向量。col: 0=X轴, 1=Y轴, 2=Z轴, 3=平移。
        /// </summary>
        public RMVector3 GetColumn(int col)
        {
            switch (col)
            {
                case 0: return new RMVector3(m00, m10, m20);
                case 1: return new RMVector3(m01, m11, m21);
                case 2: return new RMVector3(m02, m12, m22);
                case 3: return new RMVector3(m03, m13, m23);
                default: throw new IndexOutOfRangeException();
            }
        }

        /// <summary>
        /// 从旋转矩阵（3x3左上角）提取四元数。
        /// </summary>
        public RMQuaternion GetRotation()
        {
            float trace = m00 + m11 + m22;
            float x, y, z, w;

            if (trace > 0f)
            {
                float s = (float)System.Math.Sqrt(trace + 1f) * 2f;
                w = 0.25f * s;
                x = (m21 - m12) / s;
                y = (m02 - m20) / s;
                z = (m10 - m01) / s;
            }
            else if (m00 > m11 && m00 > m22)
            {
                float s = (float)System.Math.Sqrt(1f + m00 - m11 - m22) * 2f;
                w = (m21 - m12) / s;
                x = 0.25f * s;
                y = (m01 + m10) / s;
                z = (m02 + m20) / s;
            }
            else if (m11 > m22)
            {
                float s = (float)System.Math.Sqrt(1f + m11 - m00 - m22) * 2f;
                w = (m02 - m20) / s;
                x = (m01 + m10) / s;
                y = 0.25f * s;
                z = (m12 + m21) / s;
            }
            else
            {
                float s = (float)System.Math.Sqrt(1f + m22 - m00 - m11) * 2f;
                w = (m10 - m01) / s;
                x = (m02 + m20) / s;
                y = (m12 + m21) / s;
                z = 0.25f * s;
            }

            return new RMQuaternion(x, y, z, w).Normalized;
        }

        // ===== 工厂方法 =====

        /// <summary>
        /// 从平移和旋转构建齐次变换矩阵。
        /// </summary>
        public static RMMatrix4x4 FromTranslationRotation(RMVector3 pos, RMQuaternion rot)
        {
            // 将四元数转为 3x3 旋转矩阵
            float xx = rot.X * rot.X;
            float yy = rot.Y * rot.Y;
            float zz = rot.Z * rot.Z;
            float xy = rot.X * rot.Y;
            float xz = rot.X * rot.Z;
            float yz = rot.Y * rot.Z;
            float wx = rot.W * rot.X;
            float wy = rot.W * rot.Y;
            float wz = rot.W * rot.Z;

            var m = new RMMatrix4x4();
            m.m00 = 1f - 2f * (yy + zz);
            m.m10 = 2f * (xy + wz);
            m.m20 = 2f * (xz - wy);
            m.m30 = 0f;

            m.m01 = 2f * (xy - wz);
            m.m11 = 1f - 2f * (xx + zz);
            m.m21 = 2f * (yz + wx);
            m.m31 = 0f;

            m.m02 = 2f * (xz + wy);
            m.m12 = 2f * (yz - wx);
            m.m22 = 1f - 2f * (xx + yy);
            m.m32 = 0f;

            m.m03 = pos.X;
            m.m13 = pos.Y;
            m.m23 = pos.Z;
            m.m33 = 1f;

            return m;
        }

        /// <summary>
        /// 从纯平移构建齐次变换矩阵。
        /// </summary>
        public static RMMatrix4x4 FromTranslation(RMVector3 pos)
        {
            var m = Identity;
            m.m03 = pos.X;
            m.m13 = pos.Y;
            m.m23 = pos.Z;
            return m;
        }

        /// <summary>
        /// 绕任意轴旋转的齐次变换矩阵（Rodrigues 公式）。axis 需为单位向量。
        /// </summary>
        public static RMMatrix4x4 RotationAroundAxis(RMVector3 axis, float angle)
        {
            float c = (float)System.Math.Cos(angle);
            float s = (float)System.Math.Sin(angle);
            float t = 1f - c;

            float x = axis.X, y = axis.Y, z = axis.Z;

            var m = new RMMatrix4x4();
            m.m00 = t * x * x + c;
            m.m10 = t * x * y + s * z;
            m.m20 = t * x * z - s * y;

            m.m01 = t * x * y - s * z;
            m.m11 = t * y * y + c;
            m.m21 = t * y * z + s * x;

            m.m02 = t * x * z + s * y;
            m.m12 = t * y * z - s * x;
            m.m22 = t * z * z + c;

            m.m33 = 1f;
            return m;
        }

        /// <summary>
        /// 标准 DH (Denavit-Hartenberg) 齐次变换矩阵。
        /// T = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
        /// </summary>
        public static RMMatrix4x4 DHMatrix(float a, float alpha, float d, float theta)
        {
            float ct = (float)System.Math.Cos(theta);
            float st = (float)System.Math.Sin(theta);
            float ca = (float)System.Math.Cos(alpha);
            float sa = (float)System.Math.Sin(alpha);

            var m = new RMMatrix4x4();
            m.m00 = ct;         m.m01 = -st * ca;   m.m02 = st * sa;    m.m03 = a * ct;
            m.m10 = st;         m.m11 = ct * ca;     m.m12 = -ct * sa;   m.m13 = a * st;
            m.m20 = 0f;         m.m21 = sa;          m.m22 = ca;         m.m23 = d;
            m.m30 = 0f;         m.m31 = 0f;          m.m32 = 0f;         m.m33 = 1f;
            return m;
        }

        /// <summary>
        /// 绕 z 轴旋转的齐次变换矩阵（纯旋转，无平移）。
        /// 等价于 DHMatrix(0, 0, 0, angle)，但只需 2 次三角函数调用。
        /// </summary>
        public static RMMatrix4x4 RotZ(float angle)
        {
            float c = (float)System.Math.Cos(angle);
            float s = (float)System.Math.Sin(angle);

            var m = new RMMatrix4x4();
            m.m00 = c;  m.m01 = -s;
            m.m10 = s;  m.m11 = c;
            m.m22 = 1f;
            m.m33 = 1f;
            return m;
        }

        /// <summary>
        /// 沿 z 轴平移的齐次变换矩阵。
        /// </summary>
        public static RMMatrix4x4 TransZ(float d)
        {
            var m = Identity;
            m.m23 = d;
            return m;
        }

        // ===== 矩阵乘法 =====

        public static RMMatrix4x4 Multiply(RMMatrix4x4 a, RMMatrix4x4 b)
        {
            var r = new RMMatrix4x4();

            r.m00 = a.m00 * b.m00 + a.m01 * b.m10 + a.m02 * b.m20 + a.m03 * b.m30;
            r.m10 = a.m10 * b.m00 + a.m11 * b.m10 + a.m12 * b.m20 + a.m13 * b.m30;
            r.m20 = a.m20 * b.m00 + a.m21 * b.m10 + a.m22 * b.m20 + a.m23 * b.m30;
            r.m30 = a.m30 * b.m00 + a.m31 * b.m10 + a.m32 * b.m20 + a.m33 * b.m30;

            r.m01 = a.m00 * b.m01 + a.m01 * b.m11 + a.m02 * b.m21 + a.m03 * b.m31;
            r.m11 = a.m10 * b.m01 + a.m11 * b.m11 + a.m12 * b.m21 + a.m13 * b.m31;
            r.m21 = a.m20 * b.m01 + a.m21 * b.m11 + a.m22 * b.m21 + a.m23 * b.m31;
            r.m31 = a.m30 * b.m01 + a.m31 * b.m11 + a.m32 * b.m21 + a.m33 * b.m31;

            r.m02 = a.m00 * b.m02 + a.m01 * b.m12 + a.m02 * b.m22 + a.m03 * b.m32;
            r.m12 = a.m10 * b.m02 + a.m11 * b.m12 + a.m12 * b.m22 + a.m13 * b.m32;
            r.m22 = a.m20 * b.m02 + a.m21 * b.m12 + a.m22 * b.m22 + a.m23 * b.m32;
            r.m32 = a.m30 * b.m02 + a.m31 * b.m12 + a.m32 * b.m22 + a.m33 * b.m32;

            r.m03 = a.m00 * b.m03 + a.m01 * b.m13 + a.m02 * b.m23 + a.m03 * b.m33;
            r.m13 = a.m10 * b.m03 + a.m11 * b.m13 + a.m12 * b.m23 + a.m13 * b.m33;
            r.m23 = a.m20 * b.m03 + a.m21 * b.m13 + a.m22 * b.m23 + a.m23 * b.m33;
            r.m33 = a.m30 * b.m03 + a.m31 * b.m13 + a.m32 * b.m23 + a.m33 * b.m33;

            return r;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static RMMatrix4x4 operator *(RMMatrix4x4 a, RMMatrix4x4 b)
        {
            return Multiply(a, b);
        }

        /// <summary>
        /// 矩阵乘以齐次坐标向量（w=1），返回变换后的 3D 点。
        /// </summary>
        public RMVector3 MultiplyPoint(RMVector3 p)
        {
            return new RMVector3(
                m00 * p.X + m01 * p.Y + m02 * p.Z + m03,
                m10 * p.X + m11 * p.Y + m12 * p.Z + m13,
                m20 * p.X + m21 * p.Y + m22 * p.Z + m23
            );
        }

        /// <summary>
        /// 矩阵乘以方向向量（w=0），只应用旋转不应用平移。
        /// </summary>
        public RMVector3 MultiplyDirection(RMVector3 d)
        {
            return new RMVector3(
                m00 * d.X + m01 * d.Y + m02 * d.Z,
                m10 * d.X + m11 * d.Y + m12 * d.Z,
                m20 * d.X + m21 * d.Y + m22 * d.Z
            );
        }

        // ===== 转置 =====
        public static RMMatrix4x4 Transpose(RMMatrix4x4 m)
        {
            var r = new RMMatrix4x4();
            r.m00 = m.m00; r.m01 = m.m10; r.m02 = m.m20; r.m03 = m.m30;
            r.m10 = m.m01; r.m11 = m.m11; r.m12 = m.m21; r.m13 = m.m31;
            r.m20 = m.m02; r.m21 = m.m12; r.m22 = m.m22; r.m23 = m.m32;
            r.m30 = m.m03; r.m31 = m.m13; r.m32 = m.m23; r.m33 = m.m33;
            return r;
        }

        // ===== 逆矩阵（伴随矩阵法）=====
        public static RMMatrix4x4 Inverse(RMMatrix4x4 m)
        {
            float a00 = m.m00, a01 = m.m01, a02 = m.m02, a03 = m.m03;
            float a10 = m.m10, a11 = m.m11, a12 = m.m12, a13 = m.m13;
            float a20 = m.m20, a21 = m.m21, a22 = m.m22, a23 = m.m23;
            float a30 = m.m30, a31 = m.m31, a32 = m.m32, a33 = m.m33;

            float b00 = a00 * a11 - a01 * a10;
            float b01 = a00 * a12 - a02 * a10;
            float b02 = a00 * a13 - a03 * a10;
            float b03 = a01 * a12 - a02 * a11;
            float b04 = a01 * a13 - a03 * a11;
            float b05 = a02 * a13 - a03 * a12;
            float b06 = a20 * a31 - a21 * a30;
            float b07 = a20 * a32 - a22 * a30;
            float b08 = a20 * a33 - a23 * a30;
            float b09 = a21 * a32 - a22 * a31;
            float b10 = a21 * a33 - a23 * a31;
            float b11 = a22 * a33 - a23 * a32;

            float det = b00 * b11 - b01 * b10 + b02 * b09 + b03 * b08 - b04 * b07 + b05 * b06;

            if (System.Math.Abs(det) < 1e-12f)
                return Identity; // 奇异矩阵回退

            float invDet = 1f / det;

            var r = new RMMatrix4x4();
            r.m00 = (a11 * b11 - a12 * b10 + a13 * b09) * invDet;
            r.m10 = (-a10 * b11 + a12 * b08 - a13 * b07) * invDet;
            r.m20 = (a10 * b10 - a11 * b08 + a13 * b06) * invDet;
            r.m30 = (-a10 * b09 + a11 * b07 - a12 * b06) * invDet;

            r.m01 = (-a01 * b11 + a02 * b10 - a03 * b09) * invDet;
            r.m11 = (a00 * b11 - a02 * b08 + a03 * b07) * invDet;
            r.m21 = (-a00 * b10 + a01 * b08 - a03 * b06) * invDet;
            r.m31 = (a00 * b09 - a01 * b07 + a02 * b06) * invDet;

            r.m02 = (a31 * b05 - a32 * b04 + a33 * b03) * invDet;
            r.m12 = (-a30 * b05 + a32 * b02 - a33 * b01) * invDet;
            r.m22 = (a30 * b04 - a31 * b02 + a33 * b00) * invDet;
            r.m32 = (-a30 * b03 + a31 * b01 - a32 * b00) * invDet;

            r.m03 = (-a21 * b05 + a22 * b04 - a23 * b03) * invDet;
            r.m13 = (a20 * b05 - a22 * b02 + a23 * b01) * invDet;
            r.m23 = (-a20 * b04 + a21 * b02 - a23 * b00) * invDet;
            r.m33 = (a20 * b03 - a21 * b01 + a22 * b00) * invDet;

            return r;
        }

        // ===== IEquatable =====
        public bool Equals(RMMatrix4x4 other)
        {
            const float eps = 1e-6f;
            for (int r = 0; r < 4; r++)
                for (int c = 0; c < 4; c++)
                    if (System.Math.Abs(this[r, c] - other[r, c]) > eps)
                        return false;
            return true;
        }

        public override bool Equals(object obj)
        {
            return obj is RMMatrix4x4 other && Equals(other);
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + m00.GetHashCode();
                hash = hash * 31 + m11.GetHashCode();
                hash = hash * 31 + m22.GetHashCode();
                hash = hash * 31 + m33.GetHashCode();
                return hash;
            }
        }

        public override string ToString()
        {
            return $"[{m00:F3} {m01:F3} {m02:F3} {m03:F3}]\n" +
                   $"[{m10:F3} {m11:F3} {m12:F3} {m13:F3}]\n" +
                   $"[{m20:F3} {m21:F3} {m22:F3} {m23:F3}]\n" +
                   $"[{m30:F3} {m31:F3} {m32:F3} {m33:F3}]";
        }
    }
}
