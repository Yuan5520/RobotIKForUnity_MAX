using IEngine;

namespace RMCollision
{
    /// <summary>
    /// 碰撞体管理器。
    /// 为各段的物体添加/移除碰撞体，并确保拥有运动学 Rigidbody。
    /// </summary>
    public class ColliderManager
    {
        private readonly IEnginePhysics _physics;
        private JointSegment[] _segments;

        public ColliderManager(IEnginePhysics physics)
        {
            _physics = physics;
        }

        /// <summary>
        /// 确保所有段的根节点拥有运动学 Rigidbody（碰撞检测的必要条件）。
        /// 不论是否自动添加碰撞体，都需要调用。
        /// </summary>
        public void EnsureSegmentRigidbodies(JointSegment[] segments)
        {
            _segments = segments;
            for (int i = 0; i < segments.Length; i++)
            {
                _physics.EnsureKinematicRigidbody(segments[i].RootJoint);
            }
        }

        /// <summary>
        /// 选择性自动添加碰撞体。
        /// 对 hasOverride[i] = true 的段跳过（用户已手动配置），
        /// 其余段自动添加凸网格碰撞体。
        /// </summary>
        public void AutoAddCollidersSelective(JointSegment[] segments, bool[] hasOverride)
        {
            _segments = segments;
            for (int i = 0; i < segments.Length; i++)
            {
                if (hasOverride != null && i < hasOverride.Length && hasOverride[i])
                    continue;

                var segment = segments[i];
                if (segment.Members == null) continue;
                foreach (var member in segment.Members)
                {
                    if (member == null || !member.IsValid) continue;

                    var existing = _physics.GetColliders(member);
                    if (existing != null && existing.Length > 0)
                        continue;

                    _physics.AddMeshCollider(member, true);
                }
            }
        }

        /// <summary>
        /// 为所有段自动添加网格碰撞体（旧接口，全部段都自动添加）。
        /// </summary>
        public void AutoAddColliders(JointSegment[] segments)
        {
            AutoAddCollidersSelective(segments, null);
        }

        /// <summary>
        /// 启用/禁用所有段的碰撞体。
        /// </summary>
        public void SetAllCollidersEnabled(bool enabled)
        {
            if (_segments == null) return;
            foreach (var segment in _segments)
            {
                if (segment.Members == null) continue;
                foreach (var member in segment.Members)
                {
                    if (member != null && member.IsValid)
                        _physics.SetCollidersEnabled(member, enabled);
                }
            }
        }

        /// <summary>
        /// 移除所有段添加的碰撞体。
        /// </summary>
        public void RemoveAllColliders()
        {
            if (_segments == null) return;
            foreach (var segment in _segments)
            {
                if (segment.Members == null) continue;
                foreach (var member in segment.Members)
                {
                    if (member != null && member.IsValid)
                        _physics.RemoveAllColliders(member);
                }
            }
            _segments = null;
        }

        /// <summary>
        /// 获取指定段所有成员物体的碰撞体包围盒。
        /// </summary>
        public RMBounds[] GetSegmentBounds(int segmentIndex)
        {
            if (_segments == null || segmentIndex < 0 || segmentIndex >= _segments.Length)
                return new RMBounds[0];

            var segment = _segments[segmentIndex];
            if (segment.Members == null) return new RMBounds[0];

            var boundsList = new System.Collections.Generic.List<RMBounds>();
            foreach (var member in segment.Members)
            {
                if (member == null || !member.IsValid) continue;
                var bounds = _physics.GetColliderBounds(member);
                if (bounds.IsValid)
                    boundsList.Add(bounds);
            }
            return boundsList.ToArray();
        }

        /// <summary>
        /// 获取所有段的所有碰撞体包围盒。
        /// </summary>
        public RMBounds[] GetAllBounds()
        {
            if (_segments == null) return new RMBounds[0];
            var allBounds = new System.Collections.Generic.List<RMBounds>();
            for (int i = 0; i < _segments.Length; i++)
            {
                var segBounds = GetSegmentBounds(i);
                allBounds.AddRange(segBounds);
            }
            return allBounds.ToArray();
        }
    }
}
