namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// IK 求解器配置参数。
    /// </summary>
    public class IKSolverConfig
    {
        /// <summary>最大迭代次数。</summary>
        public int MaxIterations = 50;

        /// <summary>位置收敛阈值（米）。</summary>
        public float PositionThreshold = 0.001f;

        /// <summary>旋转收敛阈值（弧度）。</summary>
        public float RotationThreshold = 0.001f;

        /// <summary>阻尼因子 λ。</summary>
        public float DampingFactor = 0.5f;

        /// <summary>
        /// 位置误差权重（作用于误差向量的前 3 行）。
        /// 值越大，求解器越优先满足位置约束。默认 1.0。
        /// </summary>
        public float PositionWeight = 1.0f;

        /// <summary>
        /// 姿态误差权重（作用于误差向量的后 3 行）。
        /// 值越小，当位置/姿态冲突时越倾向放弃姿态精度以保住位置。
        /// 典型值范围 0.1 ~ 1.0，默认 0.3。
        /// </summary>
        public float RotationWeight = 0.3f;

        // ===== 加权 DLS（Phase 1）=====

        /// <summary>是否启用加权 DLS（关节空间权重矩阵 W）。关闭时退化为标准 DLS。</summary>
        public bool EnableWeightedDLS = true;

        /// <summary>
        /// 软限位激活区（占关节总行程的比例，0~0.5）。
        /// 当关节距限位小于 range*activationZone 时，权重开始增大。
        /// </summary>
        public float SoftLimitActivationZone = 0.1f;

        /// <summary>
        /// 软限位最大惩罚倍数。关节到达限位边界时，权重 = 1 + maxPenalty。
        /// </summary>
        public float SoftLimitMaxPenalty = 50f;

        // ===== 零空间中位回归（Phase 2）=====

        /// <summary>
        /// 零空间中位回归增益。仅在 SolvePositionOnly 模式下生效。
        /// 0 = 不使用零空间优化；值越大，冗余自由度越积极地趋向关节中位。
        /// </summary>
        public float NullSpaceMidRangeGain = 0.5f;

        // ===== 并行多解搜索（Phase 4）=====

        /// <summary>
        /// 多解搜索并行试验数。1 = 单解（传统模式），>1 = N 路并行搜索最优解。
        /// </summary>
        public int MultiSolutionTrials = 8;

        /// <summary>
        /// 关节安全阈值（占半行程比例，0~1）。
        /// limitFactor = d_near / half_range，低于此值视为"接近限位"。
        /// 用于四级梯队分类。
        /// </summary>
        public float GoodThreshold = 0.15f;

        /// <summary>
        /// 缓动优先系数 a（0~1）。
        /// 最终评分 = a * 缓动系数b + (1-a) * 相对解优度系数c。
        /// 默认 0.2，优先相对解优度。
        /// </summary>
        public float EasingPriorityCoefficient = 0.2f;

        /// <summary>
        /// 梯队优度下降系数 d（0~1）。
        /// 相对解优度 c = d^(tier-1)，即 T1=1, T2=d, T3=d², T4=d³。
        /// 默认 0.8。
        /// </summary>
        public float TierDecayCoefficient = 0.8f;

        /// <summary>
        /// 并行解速度预筛选阈值（弧度/秒）。
        /// 若任一关节角度差超过此值的 2 倍，该解将被排除。
        /// 由 Controller 在求解前从 maxInterpolationSpeed 同步。
        /// 0 = 不筛选。
        /// </summary>
        public float MaxJointSpeedForFilter = 0f;
    }
}
