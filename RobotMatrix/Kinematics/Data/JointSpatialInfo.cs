using RobotMatrix.Math;

namespace RobotMatrix.Kinematics
{
    /// <summary>
    /// 关节空间信息（从引擎层采集，传入核心层用于计算 DH 参数）。
    /// </summary>
    public struct JointSpatialInfo
    {
        /// <summary>关节在世界坐标系下的位置。</summary>
        public RMVector3 WorldPosition;

        /// <summary>关节在世界坐标系下的旋转。</summary>
        public RMQuaternion WorldRotation;

        /// <summary>用户微调平移偏移。</summary>
        public RMVector3 OriginOffset;

        /// <summary>用户微调旋转偏移（欧拉角，弧度）。</summary>
        public RMVector3 RotationOffset;

        /// <summary>关节运动轴在局部坐标系下的方向。</summary>
        public RMVector3 AxisDirection;
    }
}
