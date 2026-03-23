using System;
using UnityEngine;

namespace RobotMatrix.Runtime
{
    /// <summary>
    /// 单个关节的碰撞体覆盖配置。
    /// 一个关节可以包含多个 Collider 组件。
    /// </summary>
    [Serializable]
    public class JointColliderOverrideEntry
    {
        [Tooltip("该关节对应的 Collider 组件列表。\n" +
                 "直接拖入 BoxCollider / SphereCollider / MeshCollider 等组件。\n" +
                 "留空则对该关节使用自动生成。")]
        public Collider[] colliders = new Collider[0];

        /// <summary>是否有任何有效覆盖。</summary>
        public bool HasAnyValid
        {
            get
            {
                if (colliders == null) return false;
                for (int i = 0; i < colliders.Length; i++)
                {
                    if (colliders[i] != null) return true;
                }
                return false;
            }
        }
    }
}
