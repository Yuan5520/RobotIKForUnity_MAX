using System;

namespace RMCollision
{
    /// <summary>
    /// 可序列化的碰撞事件记录（用于持久化）。
    /// </summary>
    [Serializable]
    public class CollisionEvent
    {
        /// <summary>事件发生的真实时间（yyyy-MM-dd HH:mm:ss.fff）。</summary>
        public string Timestamp;

        /// <summary>碰撞段 A 索引。</summary>
        public int SegmentA;

        /// <summary>碰撞段 B 索引。</summary>
        public int SegmentB;

        /// <summary>碰撞物体 A 的名称。</summary>
        public string ColliderNameA;

        /// <summary>碰撞物体 B 的名称。</summary>
        public string ColliderNameB;

        /// <summary>碰撞评分。</summary>
        public float Score;

        /// <summary>事件类型：Enter / Stay / Exit。</summary>
        public string EventType;
    }
}
