using UnityEngine;
using RobotMatrix.Math;
using RobotMatrix.Kinematics;

namespace RobotMatrix.Data
{
    /// <summary>
    /// 单个关节的序列化配置数据。
    /// 存储在 Inspector 中可编辑的关节参数。
    /// </summary>
    [System.Serializable]
    public class SerializableJointConfig
    {
        [Tooltip("关节名称")]
        public string name = "Joint";

        [Tooltip("关节类型")]
        public JointType type = JointType.Revolute;

        [Tooltip("局部坐标系下的运动轴方向")]
        public Vector3 axisDirection = Vector3.forward;

        [Tooltip("是否启用关节限位")]
        public bool hasLimits = true;

        [Tooltip("关节下限（度）")]
        public float minLimitDeg = -90f;

        [Tooltip("关节上限（度）")]
        public float maxLimitDeg = 90f;

        [Tooltip("初始关节角度（度）")]
        public float initialAngleDeg;

        [Tooltip("坐标系原点平移微调")]
        public Vector3 originOffset;

        [Tooltip("坐标系旋转微调（欧拉角，度）")]
        public Vector3 rotationOffsetDeg;

        /// <summary>
        /// 转换为引擎无关的 JointConfig。
        /// </summary>
        public JointConfig ToJointConfig()
        {
            return new JointConfig
            {
                Type = type,
                AxisDirection = new RMVector3(axisDirection.x, axisDirection.y, axisDirection.z),
                HasLimits = hasLimits,
                MinLimit = minLimitDeg * RMMathUtils.Deg2Rad,
                MaxLimit = maxLimitDeg * RMMathUtils.Deg2Rad,
                InitialAngle = initialAngleDeg * RMMathUtils.Deg2Rad,
                OriginOffset = new RMVector3(originOffset.x, originOffset.y, originOffset.z),
                RotationOffset = new RMVector3(
                    rotationOffsetDeg.x * RMMathUtils.Deg2Rad,
                    rotationOffsetDeg.y * RMMathUtils.Deg2Rad,
                    rotationOffsetDeg.z * RMMathUtils.Deg2Rad)
            };
        }
    }

    /// <summary>
    /// 可视化配置数据。
    /// </summary>
    [System.Serializable]
    public class VisualizationConfig
    {
        [Tooltip("是否显示关节坐标轴")]
        public bool showJointAxes = true;

        [Tooltip("是否显示连杆线段")]
        public bool showLinks = true;

        [Tooltip("是否显示末端执行器标记")]
        public bool showEndEffector = true;

        [Tooltip("关节球体半径")]
        public float jointSphereRadius = 0.03f;

        [Tooltip("坐标轴长度")]
        public float axisLength = 0.1f;

        [Tooltip("末端执行器标记半径")]
        public float endEffectorRadius = 0.04f;

        [Tooltip("是否显示 DH 坐标帧（Z=旋转轴, X/Y=自动计算）")]
        public bool showDHFrames = true;

        [Tooltip("DH 坐标帧轴长度")]
        public float dhFrameAxisLength = 0.15f;

        [Header("IK 增强可视化")]

        [Tooltip("显示关节限位热力图（关节颜色随 limitFactor 渐变：绿→黄→红）")]
        public bool showLimitHeatmap = true;

        [Tooltip("显示关节权重弧线（弧度 = 360/w，仅加权 DLS 启用时有效）")]
        public bool showJointWeightArcs = true;

        [Tooltip("显示拖动补偿可视化（TCP 锚点 + 补偿关节脉冲环）")]
        public bool showDragCompensation = true;

        [Tooltip("显示所有候选解的幽灵臂（仅在多解搜索时有效）")]
        public bool showAllSolutions = true;
    }

    /// <summary>
    /// IK 求解器的序列化配置。
    /// </summary>
    [System.Serializable]
    public class SerializableIKConfig
    {
        [Tooltip("最大迭代次数")]
        [Range(10, 300)]
        public int maxIterations = 150;

        [Tooltip("位置收敛阈值（米）")]
        public float positionThreshold = 0.001f;

        [Tooltip("旋转收敛阈值（弧度）")]
        public float rotationThreshold = 0.001f;

        [Tooltip("DLS 阻尼因子")]
        [Range(0.01f, 5f)]
        public float dampingFactor = 0.5f;

        [Tooltip("位置误差权重，值越大越优先满足位置约束")]
        [Range(0.01f, 5f)]
        public float positionWeight = 1.0f;

        [Tooltip("姿态误差权重，值越小越倾向放弃姿态精度以保住位置")]
        [Range(0.01f, 5f)]
        public float rotationWeight = 0.3f;

        [Header("加权 DLS")]

        [Tooltip("启用加权 DLS（关节靠近限位时增大运动阻力）")]
        public bool enableWeightedDLS = true;

        [Tooltip("软限位激活区（占关节总行程比例，0.01~0.5）")]
        [Range(0.01f, 0.5f)]
        public float softLimitActivationZone = 0.1f;

        [Tooltip("软限位最大惩罚倍数（到达限位时权重 = 1 + maxPenalty）")]
        [Range(1f, 200f)]
        public float softLimitMaxPenalty = 50f;

        [Header("零空间中位回归")]

        [Tooltip("零空间中位回归增益（仅 SolvePositionOnly 模式生效，0 = 禁用）")]
        [Range(0f, 2f)]
        public float nullSpaceMidRangeGain = 0.5f;

        [Header("多解搜索")]

        [Tooltip("并行试验数（1 = 传统单解，>1 = N 路并行搜索最优解）")]
        [Range(1, 32)]
        public int multiSolutionTrials = 8;

        [Tooltip("关节安全阈值（limitFactor 低于此值视为接近限位，用于梯队分类）")]
        [Range(0.05f, 0.4f)]
        public float goodThreshold = 0.15f;

        [Tooltip("缓动优先系数 a（0~1）。最终评分 = a*缓动系数 + (1-a)*解优度系数。越大越偏好小动作。")]
        [Range(0f, 1f)]
        public float easingPriorityCoefficient = 0.5f;

        [Tooltip("梯队优度下降系数 d（0~1）。c = d^(tier-1)，T1=1, T2=d, T3=d², T4=d³。")]
        [Range(0.1f, 1f)]
        public float tierDecayCoefficient = 0.8f;

        /// <summary>
        /// 转换为引擎无关的 IKSolverConfig。
        /// </summary>
        public IKSolverConfig ToIKSolverConfig()
        {
            return new IKSolverConfig
            {
                MaxIterations = maxIterations,
                PositionThreshold = positionThreshold,
                RotationThreshold = rotationThreshold,
                DampingFactor = dampingFactor,
                PositionWeight = positionWeight,
                RotationWeight = rotationWeight,
                EnableWeightedDLS = enableWeightedDLS,
                SoftLimitActivationZone = softLimitActivationZone,
                SoftLimitMaxPenalty = softLimitMaxPenalty,
                NullSpaceMidRangeGain = nullSpaceMidRangeGain,
                MultiSolutionTrials = multiSolutionTrials,
                GoodThreshold = goodThreshold,
                EasingPriorityCoefficient = easingPriorityCoefficient,
                TierDecayCoefficient = tierDecayCoefficient
            };
        }
    }
}
