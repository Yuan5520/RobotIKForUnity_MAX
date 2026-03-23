namespace RobotMatrix.Math
{
    /// <summary>
    /// 几何计算工具方法集合。
    /// 提供跨模块共享的坐标系构建和轴方向计算等基础功能。
    /// </summary>
    public static class RMGeometryUtils
    {
        /// <summary>
        /// 选择一个垂直于给定轴的方向。
        /// 策略：取与 axis 最不平行的坐标基向量做叉积，保证结果远离退化。
        /// </summary>
        public static RMVector3 ChoosePerpendicularAxis(RMVector3 axis)
        {
            float absX = System.Math.Abs(axis.X);
            float absY = System.Math.Abs(axis.Y);
            float absZ = System.Math.Abs(axis.Z);

            RMVector3 reference;
            if (absX <= absY && absX <= absZ)
                reference = RMVector3.Right;
            else if (absY <= absZ)
                reference = RMVector3.Up;
            else
                reference = RMVector3.Forward;

            var perp = RMVector3.Cross(axis, reference);
            return RMVector3.Normalize(perp);
        }

        /// <summary>
        /// 从位置和三个坐标轴构建 4x4 齐次变换矩阵 [X | Y | Z | Position]。
        /// </summary>
        public static RMMatrix4x4 BuildFrameMatrix(RMVector3 position,
                                                    RMVector3 xAxis,
                                                    RMVector3 yAxis,
                                                    RMVector3 zAxis)
        {
            var m = new RMMatrix4x4();
            m.m00 = xAxis.X; m.m01 = yAxis.X; m.m02 = zAxis.X; m.m03 = position.X;
            m.m10 = xAxis.Y; m.m11 = yAxis.Y; m.m12 = zAxis.Y; m.m13 = position.Y;
            m.m20 = xAxis.Z; m.m21 = yAxis.Z; m.m22 = zAxis.Z; m.m23 = position.Z;
            m.m30 = 0f;      m.m31 = 0f;      m.m32 = 0f;      m.m33 = 1f;
            return m;
        }
    }
}
