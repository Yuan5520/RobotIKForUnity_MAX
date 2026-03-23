using IEngine;

namespace RobotMatrix.Controller
{
    /// <summary>
    /// 关节层级关系类型。
    /// </summary>
    public enum JointRelationType
    {
        /// <summary>
        /// 前一运动学关节是当前关节的场景树祖先（层级级联有效，使用增量旋转法）。
        /// </summary>
        AncestorDescendant,

        /// <summary>
        /// 前一运动学关节不是当前关节的场景树祖先（级联无效，需世界坐标统一法）。
        /// </summary>
        Independent
    }

    /// <summary>
    /// 单个关节的层级分析结果。
    /// </summary>
    public struct JointHierarchyInfo
    {
        /// <summary>关节在配置中的序号。</summary>
        public int JointIndex;

        /// <summary>运动学链中上一关节索引（-1 = 基座根节点）。</summary>
        public int ParentJointIndex;

        /// <summary>与上一运动学关节的场景树层级关系。</summary>
        public JointRelationType Relation;
    }

    /// <summary>
    /// 关节层级分析器。
    /// 动态解析各关节 IEngineObject 的实际场景树层级关系，
    /// 以确定每个关节使用增量旋转法还是世界坐标统一法进行场景同步。
    /// </summary>
    public static class JointHierarchyAnalyzer
    {
        /// <summary>
        /// 分析关节数组的实际层级关系。
        /// 第一个关节（index=0）的 Relation 始终为 AncestorDescendant。
        /// </summary>
        public static JointHierarchyInfo[] Analyze(IEngineObject[] jointObjects)
        {
            int n = jointObjects.Length;
            var infos = new JointHierarchyInfo[n];

            for (int i = 0; i < n; i++)
            {
                infos[i].JointIndex = i;
                infos[i].ParentJointIndex = i - 1;

                if (i == 0)
                {
                    // 第一个关节无前驱，始终标记为 AncestorDescendant
                    infos[i].ParentJointIndex = -1;
                    infos[i].Relation = JointRelationType.AncestorDescendant;
                }
                else
                {
                    // 检测 jointObjects[i-1] 是否为 jointObjects[i] 的场景树祖先
                    if (IsAncestorOf(jointObjects[i - 1], jointObjects[i]))
                        infos[i].Relation = JointRelationType.AncestorDescendant;
                    else
                        infos[i].Relation = JointRelationType.Independent;
                }
            }

            return infos;
        }

        /// <summary>
        /// 判断 ancestor 是否为 descendant 的场景树祖先。
        /// 沿 descendant.Parent 链逐级上溯，若遇到 ancestor 则返回 true。
        /// </summary>
        public static bool IsAncestorOf(IEngineObject ancestor, IEngineObject descendant)
        {
            if (ancestor == null || descendant == null)
                return false;

            var current = descendant.Parent;
            // 设置上限防止死循环（场景树不应过深）
            int maxDepth = 100;
            int depth = 0;

            while (current != null && depth < maxDepth)
            {
                if (current.Equals(ancestor))
                    return true;
                current = current.Parent;
                depth++;
            }

            return false;
        }
    }
}
