using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 碰撞事件数据（引擎无关）。
    /// </summary>
    public struct RMCollisionEventData
    {
        /// <summary>自身物体。</summary>
        public IEngineObject Self;

        /// <summary>碰撞对方物体。</summary>
        public IEngineObject Other;

        /// <summary>接触点数组（世界坐标）。</summary>
        public RMVector3[] ContactPoints;

        /// <summary>碰撞相对速度。</summary>
        public RMVector3 RelativeVelocity;

        /// <summary>估算接触面积。</summary>
        public float ContactArea;

        /// <summary>平均碰撞法线。</summary>
        public RMVector3 ContactNormal;
    }
}
