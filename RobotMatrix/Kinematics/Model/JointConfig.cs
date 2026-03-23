using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 关节类型枚举。
    /// </summary>
    public enum JointType
    {
        /// <summary>旋转关节（绕轴旋转）。</summary>
        Revolute,
        /// <summary>移动关节（沿轴平移）。</summary>
        Prismatic
    }

    /// <summary>
    /// 标准 DH 参数（Denavit-Hartenberg）。
    /// </summary>
    public struct DHParameters
    {
        /// <summary>连杆长度（沿 x_i 轴）。</summary>
        public float A;
        /// <summary>连杆扭转角（绕 x_i 轴，弧度）。</summary>
        public float Alpha;
        /// <summary>连杆偏距（沿 z_{i-1} 轴）。</summary>
        public float D;
        /// <summary>关节角偏移（旋转关节的初始偏移）。</summary>
        public float ThetaOffset;

        public DHParameters(float a, float alpha, float d, float thetaOffset)
        {
            A = a;
            Alpha = alpha;
            D = d;
            ThetaOffset = thetaOffset;
        }

        public override string ToString()
        {
            return $"DH(a={A:F4}, alpha={Alpha:F4}, d={D:F4}, theta_off={ThetaOffset:F4})";
        }
    }

    /// <summary>
    /// 单个关节的配置数据（纯数据，引擎无关）。
    /// </summary>
    public class JointConfig
    {
        /// <summary>关节类型。</summary>
        public JointType Type;

        /// <summary>局部坐标系下的运动轴方向（默认 Z 轴）。</summary>
        public RMVector3 AxisDirection = RMVector3.Forward;

        /// <summary>运动下限（float.NegativeInfinity = 不限制）。</summary>
        public float MinLimit = float.NegativeInfinity;

        /// <summary>运动上限（float.PositiveInfinity = 不限制）。</summary>
        public float MaxLimit = float.PositiveInfinity;

        /// <summary>是否启用限位。</summary>
        public bool HasLimits;

        /// <summary>坐标系原点平移微调。</summary>
        public RMVector3 OriginOffset;

        /// <summary>坐标系旋转微调（欧拉角，弧度）。</summary>
        public RMVector3 RotationOffset;

        /// <summary>初始关节角度/偏移。</summary>
        public float InitialAngle;

        /// <summary>自动计算得出的 DH 参数（仅用于调试显示）。</summary>
        public DHParameters DH;

        /// <summary>
        /// 从上一关节 DH 帧（旋转后）到当前关节 DH 帧（旋转前）的完整固定变换。
        /// 替代 DH 四参数用于 FK 计算，能精确描述任意关节空间布局（无残差丢失）。
        /// FK 链：T *= LinkTransform * RotZ(q_i) （旋转关节）
        /// 先变换到当前帧，再绕当前帧的 z 轴旋转，确保关节 i 绕自身物理轴转动。
        /// </summary>
        public RMMatrix4x4 LinkTransform = RMMatrix4x4.Identity;
    }
}
