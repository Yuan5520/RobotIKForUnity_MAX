using IEngine;

namespace RMCollision
{
    /// <summary>
    /// 关节段：两个相邻关节之间的所有物体集合。
    /// 用于碰撞检测的分组管理。
    /// </summary>
    public class JointSegment
    {
        /// <summary>段索引（与关节索引对应）。</summary>
        public int SegmentIndex;

        /// <summary>该段的根关节物体。</summary>
        public IEngineObject RootJoint;

        /// <summary>该段包含的所有物体（包括根关节及其下的子物体，截止到下一个关节）。</summary>
        public IEngineObject[] Members;
    }
}
