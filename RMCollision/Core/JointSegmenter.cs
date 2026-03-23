using System.Collections.Generic;
using IEngine;

namespace RMCollision
{
    /// <summary>
    /// 关节分段器。
    /// 将机械臂按关节边界分割为多个段，每段包含一个关节根节点及其下的所有子物体
    /// （截止到下一个关节根节点）。
    /// </summary>
    public static class JointSegmenter
    {
        /// <summary>
        /// 根据关节物体数组进行分段。
        /// 使用 IEnginePhysics.GetDescendantsUntil 获取每段的成员物体。
        /// </summary>
        /// <param name="jointObjects">按运动学链排列的关节物体数组。</param>
        /// <param name="physics">物理引擎抽象接口。</param>
        /// <returns>分段数组，长度与 jointObjects 相同。</returns>
        public static JointSegment[] Segment(IEngineObject[] jointObjects, IEnginePhysics physics)
        {
            int n = jointObjects.Length;
            var segments = new JointSegment[n];

            for (int i = 0; i < n; i++)
            {
                var segment = new JointSegment
                {
                    SegmentIndex = i,
                    RootJoint = jointObjects[i]
                };

                if (i < n - 1)
                {
                    // 获取该关节下的所有后代，但在遇到下一个关节时停止
                    segment.Members = physics.GetDescendantsUntil(jointObjects[i], jointObjects[i + 1]);
                }
                else
                {
                    // 最后一个关节：获取所有后代
                    segment.Members = physics.GetAllDescendants(jointObjects[i]);
                }

                segments[i] = segment;
            }

            return segments;
        }

        /// <summary>
        /// 构建物体到段索引的反查字典。
        /// 通过物体名称作为 key（IEngineObject 无唯一 ID，用名称近似）。
        /// </summary>
        public static Dictionary<string, int> BuildObjectToSegmentMap(JointSegment[] segments)
        {
            var map = new Dictionary<string, int>();
            for (int i = 0; i < segments.Length; i++)
            {
                if (segments[i].Members == null) continue;
                foreach (var obj in segments[i].Members)
                {
                    if (obj != null && obj.IsValid)
                    {
                        string key = obj.Name;
                        if (!map.ContainsKey(key))
                            map[key] = i;
                    }
                }
            }
            return map;
        }
    }
}
