namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// IK 求解结果。
    /// </summary>
    public class IKResult
    {
        /// <summary>求解后的关节角度数组。</summary>
        public float[] JointAngles;

        /// <summary>是否收敛到目标精度。</summary>
        public bool Converged;

        /// <summary>最终位置误差（米）。</summary>
        public float PositionError;

        /// <summary>最终旋转误差（弧度）。</summary>
        public float RotationError;

        /// <summary>实际使用的迭代次数。</summary>
        public int IterationsUsed;

        /// <summary>
        /// 解质量梯队（1~4，越低越好）。
        ///   Tier 1: 所有关节远离限位（全安全）
        ///   Tier 2: 少于一半关节接近限位
        ///   Tier 3: 超过一半关节接近限位
        ///   Tier 4: 有关节触碰硬限位
        /// </summary>
        public int SolutionTier;

        /// <summary>
        /// 梯队内得分（0~1，越高越好）。
        /// 等于所有关节 limitFactor 的平均值。
        /// </summary>
        public float SolutionScore;

        /// <summary>
        /// 缓动系数 b（0~1，越高表示相对当前姿态移动越少）。
        /// b = 1 / (1 + rotationCost)，rotationCost = Σ(|Δθ_j| / range_j)。
        /// </summary>
        public float EasingCoefficient;

        /// <summary>
        /// 综合评分 = a * b + (1-a) * c。
        /// a = 缓动优先系数, b = 缓动系数, c = 相对解优度系数（梯队映射）。
        /// </summary>
        public float CompositeScore;
    }
}
