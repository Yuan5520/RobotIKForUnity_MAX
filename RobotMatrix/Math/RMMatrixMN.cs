using System;

namespace RobotMatrix.Math
{
    /// <summary>
    /// 任意维度矩阵（M x N），一维行优先存储。
    /// 服务于雅可比矩阵和 DLS 求解中的矩阵运算。
    /// </summary>
    public class RMMatrixMN
    {
        private readonly float[] _data;

        public int Rows { get; }
        public int Cols { get; }

        public RMMatrixMN(int rows, int cols)
        {
            Rows = rows;
            Cols = cols;
            _data = new float[rows * cols];
        }

        public float this[int row, int col]
        {
            get => _data[row * Cols + col];
            set => _data[row * Cols + col] = value;
        }

        // ===== 静态工厂 =====

        /// <summary>
        /// 创建 N x N 单位矩阵。
        /// </summary>
        public static RMMatrixMN Identity(int size)
        {
            var m = new RMMatrixMN(size, size);
            for (int i = 0; i < size; i++)
                m[i, i] = 1f;
            return m;
        }

        /// <summary>
        /// 从列向量快速构建 6 x N 雅可比矩阵。
        /// 6 = 任务空间维度（3 位置 + 3 姿态），与机械臂关节数无关。
        /// N = 关节数量（由 upper/lower 数组长度决定），支持任意 DOF 串联臂。
        /// 例如：3-DOF → 6x3，6-DOF → 6x6，7-DOF → 6x7。
        /// upper[i] = J_pos 的第 i 列 (3x1)，lower[i] = J_rot 的第 i 列 (3x1)。
        /// </summary>
        public static RMMatrixMN FromColumnVectors(RMVector3[] upper, RMVector3[] lower)
        {
            int n = upper.Length;
            var m = new RMMatrixMN(6, n);

            for (int i = 0; i < n; i++)
            {
                m[0, i] = upper[i].X;
                m[1, i] = upper[i].Y;
                m[2, i] = upper[i].Z;
                m[3, i] = lower[i].X;
                m[4, i] = lower[i].Y;
                m[5, i] = lower[i].Z;
            }

            return m;
        }

        // ===== 矩阵运算 =====

        /// <summary>
        /// 矩阵乘法 A(M x K) * B(K x N) -> C(M x N)。
        /// </summary>
        public static RMMatrixMN Multiply(RMMatrixMN a, RMMatrixMN b)
        {
            if (a.Cols != b.Rows)
                throw new ArgumentException(
                    $"Matrix dimensions mismatch: ({a.Rows}x{a.Cols}) * ({b.Rows}x{b.Cols})");

            var result = new RMMatrixMN(a.Rows, b.Cols);
            int m = a.Rows, k = a.Cols, n = b.Cols;

            for (int i = 0; i < m; i++)
            {
                int aRowOffset = i * k;
                int rRowOffset = i * n;
                for (int j = 0; j < n; j++)
                {
                    float sum = 0f;
                    for (int p = 0; p < k; p++)
                        sum += a._data[aRowOffset + p] * b._data[p * n + j];
                    result._data[rRowOffset + j] = sum;
                }
            }

            return result;
        }

        /// <summary>
        /// 矩阵乘向量 M(M x N) * v(N x 1) -> result(M x 1)。
        /// </summary>
        public static float[] MultiplyVector(RMMatrixMN m, float[] v)
        {
            if (m.Cols != v.Length)
                throw new ArgumentException(
                    $"Dimension mismatch: ({m.Rows}x{m.Cols}) * vec({v.Length})");

            var result = new float[m.Rows];
            int rows = m.Rows, cols = m.Cols;

            for (int i = 0; i < rows; i++)
            {
                float sum = 0f;
                int offset = i * cols;
                for (int j = 0; j < cols; j++)
                    sum += m._data[offset + j] * v[j];
                result[i] = sum;
            }

            return result;
        }

        /// <summary>
        /// 转置 M(M x N) -> result(N x M)。
        /// </summary>
        public static RMMatrixMN Transpose(RMMatrixMN m)
        {
            var result = new RMMatrixMN(m.Cols, m.Rows);
            for (int i = 0; i < m.Rows; i++)
                for (int j = 0; j < m.Cols; j++)
                    result[j, i] = m[i, j];
            return result;
        }

        /// <summary>
        /// 矩阵加法（同维度）。
        /// </summary>
        public static RMMatrixMN Add(RMMatrixMN a, RMMatrixMN b)
        {
            if (a.Rows != b.Rows || a.Cols != b.Cols)
                throw new ArgumentException("Matrix dimensions must match for addition.");

            var result = new RMMatrixMN(a.Rows, a.Cols);
            int len = a._data.Length;
            for (int i = 0; i < len; i++)
                result._data[i] = a._data[i] + b._data[i];
            return result;
        }

        /// <summary>
        /// 标量乘法。
        /// </summary>
        public static RMMatrixMN ScalarMultiply(RMMatrixMN m, float s)
        {
            var result = new RMMatrixMN(m.Rows, m.Cols);
            int len = m._data.Length;
            for (int i = 0; i < len; i++)
                result._data[i] = m._data[i] * s;
            return result;
        }

        /// <summary>
        /// 优化方法：A^T * v，无需显式构造转置矩阵。
        /// A 为 (M x N)，v 为 (M x 1)，结果为 (N x 1)。
        /// </summary>
        public static float[] TransposeMultiply(RMMatrixMN a, float[] v)
        {
            if (a.Rows != v.Length)
                throw new ArgumentException(
                    $"Dimension mismatch: ({a.Rows}x{a.Cols})^T * vec({v.Length})");

            var result = new float[a.Cols];
            int rows = a.Rows, cols = a.Cols;

            for (int j = 0; j < cols; j++)
            {
                float sum = 0f;
                for (int i = 0; i < rows; i++)
                    sum += a._data[i * cols + j] * v[i];
                result[j] = sum;
            }

            return result;
        }

        /// <summary>
        /// 专用 6x6 矩阵求逆（高斯消元 + 部分主元选择）。
        /// DLS 公式中 (J*J^T + lambda^2*I) 的结果始终为 6x6：
        ///   J 为 6xN（6 = 任务空间维度，N = 关节数），
        ///   J*J^T 为 (6xN)*(Nx6) = 6x6，无论 N 是多少。
        /// 因此此方法适用于任意 DOF 的串联机械臂。
        /// 返回逆矩阵；若奇异则返回 null。
        /// </summary>
        public static RMMatrixMN Inverse6x6(RMMatrixMN m)
        {
            if (m.Rows != 6 || m.Cols != 6)
                throw new ArgumentException("Inverse6x6 requires a 6x6 matrix.");

            return InverseNxN(m);
        }

        /// <summary>
        /// 提取指定行范围的子矩阵。
        /// </summary>
        public RMMatrixMN SubRows(int startRow, int count)
        {
            var result = new RMMatrixMN(count, Cols);
            for (int i = 0; i < count; i++)
            {
                int srcOffset = (startRow + i) * Cols;
                int dstOffset = i * Cols;
                Array.Copy(_data, srcOffset, result._data, dstOffset, Cols);
            }
            return result;
        }

        /// <summary>
        /// 通用 NxN 矩阵求逆（高斯消元 + 部分主元选择）。
        /// 返回逆矩阵；若奇异则返回 null。
        /// </summary>
        public static RMMatrixMN InverseNxN(RMMatrixMN m)
        {
            if (m.Rows != m.Cols)
                throw new ArgumentException("Matrix must be square for inversion.");

            int n = m.Rows;
            float[,] aug = new float[n, n * 2];

            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                    aug[i, j] = m[i, j];
                aug[i, i + n] = 1f;
            }

            for (int col = 0; col < n; col++)
            {
                int maxRow = col;
                float maxVal = System.Math.Abs(aug[col, col]);
                for (int row = col + 1; row < n; row++)
                {
                    float val = System.Math.Abs(aug[row, col]);
                    if (val > maxVal) { maxVal = val; maxRow = row; }
                }

                if (maxVal < 1e-10f) return null;

                if (maxRow != col)
                {
                    for (int j = 0; j < n * 2; j++)
                    {
                        float temp = aug[col, j];
                        aug[col, j] = aug[maxRow, j];
                        aug[maxRow, j] = temp;
                    }
                }

                float invPivot = 1f / aug[col, col];
                for (int j = 0; j < n * 2; j++)
                    aug[col, j] *= invPivot;

                for (int row = 0; row < n; row++)
                {
                    if (row == col) continue;
                    float factor = aug[row, col];
                    if (System.Math.Abs(factor) < 1e-15f) continue;
                    for (int j = 0; j < n * 2; j++)
                        aug[row, j] -= factor * aug[col, j];
                }
            }

            var result = new RMMatrixMN(n, n);
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    result[i, j] = aug[i, j + n];

            return result;
        }

        /// <summary>
        /// 将矩阵内容复制到目标矩阵（避免 GC 分配）。
        /// 要求维度一致。
        /// </summary>
        public void CopyTo(RMMatrixMN target)
        {
            if (target.Rows != Rows || target.Cols != Cols)
                throw new ArgumentException("Target matrix dimensions must match.");
            Array.Copy(_data, target._data, _data.Length);
        }

        /// <summary>
        /// 清零矩阵所有元素。
        /// </summary>
        public void Clear()
        {
            Array.Clear(_data, 0, _data.Length);
        }

        // ===== In-place 运算方法（零 GC 分配版本） =====

        /// <summary>
        /// 矩阵乘法写入预分配目标：A(M x K) * B(K x N) -> result(M x N)。
        /// </summary>
        public static void MultiplyInto(RMMatrixMN a, RMMatrixMN b, RMMatrixMN result)
        {
            int m = a.Rows, k = a.Cols, n = b.Cols;
            for (int i = 0; i < m; i++)
            {
                int aRowOffset = i * k;
                int rRowOffset = i * n;
                for (int j = 0; j < n; j++)
                {
                    float sum = 0f;
                    for (int p = 0; p < k; p++)
                        sum += a._data[aRowOffset + p] * b._data[p * n + j];
                    result._data[rRowOffset + j] = sum;
                }
            }
        }

        /// <summary>
        /// 矩阵加法写入预分配目标。
        /// </summary>
        public static void AddInto(RMMatrixMN a, RMMatrixMN b, RMMatrixMN result)
        {
            int len = a._data.Length;
            for (int i = 0; i < len; i++)
                result._data[i] = a._data[i] + b._data[i];
        }

        /// <summary>
        /// 标量乘法写入预分配目标。
        /// </summary>
        public static void ScalarMultiplyInto(RMMatrixMN m, float s, RMMatrixMN result)
        {
            int len = m._data.Length;
            for (int i = 0; i < len; i++)
                result._data[i] = m._data[i] * s;
        }

        /// <summary>
        /// 转置写入预分配目标：M(M x N) -> result(N x M)。
        /// </summary>
        public static void TransposeInto(RMMatrixMN m, RMMatrixMN result)
        {
            int rows = m.Rows, cols = m.Cols;
            for (int i = 0; i < rows; i++)
                for (int j = 0; j < cols; j++)
                    result._data[j * rows + i] = m._data[i * cols + j];
        }

        /// <summary>
        /// 矩阵乘向量写入预分配目标：M(M x N) * v(N) -> result(M)。
        /// </summary>
        public static void MultiplyVectorInto(RMMatrixMN m, float[] v, float[] result)
        {
            int rows = m.Rows, cols = m.Cols;
            for (int i = 0; i < rows; i++)
            {
                float sum = 0f;
                int offset = i * cols;
                for (int j = 0; j < cols; j++)
                    sum += m._data[offset + j] * v[j];
                result[i] = sum;
            }
        }

        /// <summary>
        /// A^T * v 写入预分配目标：A(M x N)^T * v(M) -> result(N)。
        /// </summary>
        public static void TransposeMultiplyInto(RMMatrixMN a, float[] v, float[] result)
        {
            int rows = a.Rows, cols = a.Cols;
            for (int j = 0; j < cols; j++)
            {
                float sum = 0f;
                for (int i = 0; i < rows; i++)
                    sum += a._data[i * cols + j] * v[i];
                result[j] = sum;
            }
        }

        /// <summary>
        /// NxN 矩阵求逆写入预分配目标（复用增广矩阵缓冲）。
        /// 返回 true 表示成功；false 表示奇异矩阵。
        /// </summary>
        public static bool InverseNxNInto(RMMatrixMN m, RMMatrixMN result, float[,] augBuffer)
        {
            int n = m.Rows;
            int n2 = n * 2;

            // 初始化增广矩阵 [A | I]
            for (int i = 0; i < n; i++)
            {
                for (int j = 0; j < n; j++)
                    augBuffer[i, j] = m[i, j];
                for (int j = n; j < n2; j++)
                    augBuffer[i, j] = 0f;
                augBuffer[i, i + n] = 1f;
            }

            // 高斯消元 + 部分主元选择
            for (int col = 0; col < n; col++)
            {
                int maxRow = col;
                float maxVal = System.Math.Abs(augBuffer[col, col]);
                for (int row = col + 1; row < n; row++)
                {
                    float val = System.Math.Abs(augBuffer[row, col]);
                    if (val > maxVal) { maxVal = val; maxRow = row; }
                }

                if (maxVal < 1e-10f) return false;

                if (maxRow != col)
                {
                    for (int j = 0; j < n2; j++)
                    {
                        float temp = augBuffer[col, j];
                        augBuffer[col, j] = augBuffer[maxRow, j];
                        augBuffer[maxRow, j] = temp;
                    }
                }

                float invPivot = 1f / augBuffer[col, col];
                for (int j = 0; j < n2; j++)
                    augBuffer[col, j] *= invPivot;

                for (int row = 0; row < n; row++)
                {
                    if (row == col) continue;
                    float factor = augBuffer[row, col];
                    if (System.Math.Abs(factor) < 1e-15f) continue;
                    for (int j = 0; j < n2; j++)
                        augBuffer[row, j] -= factor * augBuffer[col, j];
                }
            }

            // 提取逆矩阵
            for (int i = 0; i < n; i++)
                for (int j = 0; j < n; j++)
                    result[i, j] = augBuffer[i, j + n];

            return true;
        }

        /// <summary>
        /// 对列进行对角缩放：target[r,c] = source[r,c] * colScale[c]。
        /// 等价于 source * diag(colScale)，但无需构造 N×N 对角矩阵，
        /// 时间复杂度 O(M*N)，零 GC 分配。
        /// 用于加权 DLS 中的列缩放雅可比 Jw = J * W^{-1}。
        /// </summary>
        public static void DiagScaleColumnsInto(RMMatrixMN source, float[] colScale, RMMatrixMN target)
        {
            int rows = source.Rows, cols = source.Cols;
            for (int r = 0; r < rows; r++)
            {
                int offset = r * cols;
                for (int c = 0; c < cols; c++)
                    target._data[offset + c] = source._data[offset + c] * colScale[c];
            }
        }

        /// <summary>
        /// 从列向量写入预分配的 6 x N 矩阵。
        /// </summary>
        public static void FromColumnVectorsInto(RMVector3[] upper, RMVector3[] lower, RMMatrixMN target)
        {
            int n = upper.Length;
            for (int i = 0; i < n; i++)
            {
                target[0, i] = upper[i].X;
                target[1, i] = upper[i].Y;
                target[2, i] = upper[i].Z;
                target[3, i] = lower[i].X;
                target[4, i] = lower[i].Y;
                target[5, i] = lower[i].Z;
            }
        }

        /// <summary>
        /// 提取指定行范围写入预分配的子矩阵。
        /// </summary>
        public void SubRowsInto(int startRow, int count, RMMatrixMN target)
        {
            for (int i = 0; i < count; i++)
            {
                int srcOffset = (startRow + i) * Cols;
                int dstOffset = i * Cols;
                Array.Copy(_data, srcOffset, target._data, dstOffset, Cols);
            }
        }

        public override string ToString()
        {
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < Rows; i++)
            {
                sb.Append("[");
                for (int j = 0; j < Cols; j++)
                {
                    if (j > 0) sb.Append(", ");
                    sb.Append(this[i, j].ToString("F4"));
                }
                sb.Append("]\n");
            }
            return sb.ToString();
        }
    }
}
