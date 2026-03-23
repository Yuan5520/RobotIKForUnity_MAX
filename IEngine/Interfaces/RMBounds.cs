using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 碰撞体的轴对齐包围盒信息（引擎无关）。
    /// </summary>
    public struct RMBounds
    {
        /// <summary>包围盒世界坐标中心。</summary>
        public RMVector3 Center;

        /// <summary>包围盒全尺寸（宽/高/深，非半径）。</summary>
        public RMVector3 Size;

        /// <summary>是否有效（目标物体是否拥有碰撞体）。</summary>
        public bool IsValid;

        public RMBounds(RMVector3 center, RMVector3 size)
        {
            Center = center;
            Size = size;
            IsValid = true;
        }

        public static readonly RMBounds Invalid = new RMBounds { IsValid = false };
    }
}
