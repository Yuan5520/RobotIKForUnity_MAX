using System.Collections.Generic;
using IEngine;
using RMPersistence;
using RobotMatrix.Math;
using UnityEngine;

namespace RMCollision
{
    /// <summary>
    /// 碰撞检测套件控制器。
    /// 顶层入口，协调分段、碰撞体管理、校准、检测和可视化。
    /// </summary>
    public class CollisionSuiteController
    {
        private CollisionConfig _config;
        private IEnginePhysics _physics;
        private IEngineObject[] _jointObjects;

        private JointSegment[] _segments;
        private int _externalSegmentIndex = -1; // 外部碰撞体的虚拟段索引
        private ColliderManager _colliderManager;
        private StaticInterferenceCalibrator _calibrator;
        private CollisionDetector _detector;
        private bool _initialized;

        /// <summary>是否已初始化。</summary>
        public bool IsInitialized => _initialized;

        /// <summary>碰撞检测器（供外部查询评分等信息）。</summary>
        public CollisionDetector Detector => _detector;

        /// <summary>碰撞体管理器（供 Gizmo 获取碰撞体边界）。</summary>
        public ColliderManager ColliderMgr => _colliderManager;

        /// <summary>分段数据（供 Gizmo 和调试使用）。</summary>
        public JointSegment[] Segments => _segments;

        /// <summary>外部碰撞体所属的虚拟段索引。-1 表示无外部碰撞体。</summary>
        public int ExternalSegmentIndex => _externalSegmentIndex;

        /// <summary>
        /// 初始化碰撞检测套件。
        /// </summary>
        /// <param name="jointObjects">关节物体数组。</param>
        /// <param name="physics">物理引擎抽象。</param>
        /// <param name="config">碰撞检测配置。</param>
        /// <param name="persistenceConfig">持久化配置（可选）。</param>
        /// <param name="colliderOverrides">
        /// 手动碰撞体覆盖数组（可选）。外层长度应与 jointObjects 一致。
        /// 每个元素是该关节对应的多个碰撞体覆盖物体数组。
        /// 非空的元素表示该关节使用用户手动配置的碰撞体，
        /// 为空的元素根据 AutoAddColliders 决定是否自动生成。
        /// </param>
        /// <param name="externalColliders">
        /// 外部环境碰撞体数组（可选）。TCP 工具、工件、桌面等不属于机器人的碰撞体。
        /// 系统会自动设置 isTrigger 和 Rigidbody，并在校准阶段排除初始接触。
        /// </param>
        public void Initialize(IEngineObject[] jointObjects, IEnginePhysics physics,
                                CollisionConfig config, PersistenceConfig persistenceConfig = null,
                                IEngineObject[][] colliderOverrides = null,
                                IEngineObject[] externalColliders = null)
        {
            _jointObjects = jointObjects;
            _physics = physics;
            _config = config ?? new CollisionConfig();

            if (!_config.Enabled)
            {
                _initialized = false;
                return;
            }

            // 1. 关节分段
            _segments = JointSegmenter.Segment(jointObjects, physics);
            var objectToSegment = JointSegmenter.BuildObjectToSegmentMap(_segments);

            // 2. 碰撞体管理
            _colliderManager = new ColliderManager(physics);

            // 处理手动碰撞体覆盖：将 Collider 所在物体加入对应段，并注册到段映射
            if (colliderOverrides != null)
            {
                for (int i = 0; i < _segments.Length && i < colliderOverrides.Length; i++)
                {
                    if (colliderOverrides[i] == null || colliderOverrides[i].Length == 0) continue;

                    var combined = new List<IEngineObject>(_segments[i].Members ?? new IEngineObject[0]);

                    // 遍历该关节的所有 Collider 覆盖（每个元素是 Collider 所在 GameObject 的 IEngineObject）
                    for (int j = 0; j < colliderOverrides[i].Length; j++)
                    {
                        var colliderObj = colliderOverrides[i][j];
                        if (colliderObj == null || !colliderObj.IsValid) continue;

                        // 直接注册 Collider 所在物体到段映射（无需遍历后代，引用的就是具体碰撞体）
                        if (!objectToSegment.ContainsKey(colliderObj.Name))
                            objectToSegment[colliderObj.Name] = i;

                        // Collider 可能不在关节层级下，需确保其自身拥有运动学 Rigidbody
                        physics.EnsureKinematicRigidbody(colliderObj);

                        // 确保手动覆盖的 Collider 为 Trigger 模式（kinematic 之间只有 Trigger 能触发）
                        physics.SetCollidersIsTrigger(colliderObj, true);

                        // 加入段成员
                        combined.Add(colliderObj);
                    }

                    _segments[i].Members = combined.ToArray();
                }
            }

            // 为每个段确保运动学 Rigidbody（不论是否自动添加碰撞体）
            _colliderManager.EnsureSegmentRigidbodies(_segments);

            // 自动添加碰撞体（仅对没有手动覆盖的段）
            if (_config.AutoAddColliders)
            {
                bool[] hasOverride = new bool[_segments.Length];
                if (colliderOverrides != null)
                {
                    for (int i = 0; i < _segments.Length && i < colliderOverrides.Length; i++)
                        hasOverride[i] = colliderOverrides[i] != null && colliderOverrides[i].Length > 0;
                }
                _colliderManager.AutoAddCollidersSelective(_segments, hasOverride);
            }

            // 3. 外部环境碰撞体：作为虚拟段加入检测
            _externalSegmentIndex = _segments.Length; // 紧跟关节段之后
            if (externalColliders != null && externalColliders.Length > 0)
            {
                var extMembers = new List<IEngineObject>();
                foreach (var extObj in externalColliders)
                {
                    if (extObj == null || !extObj.IsValid) continue;

                    // 注册到段映射
                    if (!objectToSegment.ContainsKey(extObj.Name))
                        objectToSegment[extObj.Name] = _externalSegmentIndex;

                    // 确保 Rigidbody + isTrigger
                    physics.EnsureKinematicRigidbody(extObj);
                    physics.SetCollidersIsTrigger(extObj, true);

                    extMembers.Add(extObj);
                }

                if (extMembers.Count > 0)
                {
                    // 扩展 _segments 数组，追加外部虚拟段
                    var extSegment = new JointSegment
                    {
                        RootJoint = extMembers[0], // 用第一个外部物体作为根（仅供 Gizmo 参考）
                        Members = extMembers.ToArray()
                    };
                    var newSegments = new JointSegment[_segments.Length + 1];
                    System.Array.Copy(_segments, newSegments, _segments.Length);
                    newSegments[_segments.Length] = extSegment;
                    _segments = newSegments;
                }
            }

            // 4. 静态干涉校准器
            _calibrator = new StaticInterferenceCalibrator(_config.CalibrationDurationSec);

            // 4. 持久化会话
            IPersistenceSession persistence = null;
            if (_config.EnablePersistence)
                persistence = PersistenceManager.CreateSession("RMCollision", persistenceConfig);

            // 5. 碰撞检测器
            _detector = new CollisionDetector(_config, objectToSegment, _calibrator, persistence);
            _detector.OnCollisionDetected += OnCollisionAlert;

            // 6. 为所有段的成员确保 isTrigger 并注册碰撞监听
            foreach (var segment in _segments)
            {
                if (segment.Members == null) continue;
                foreach (var member in segment.Members)
                {
                    if (member != null && member.IsValid)
                    {
                        physics.SetCollidersIsTrigger(member, true);
                        physics.RegisterCollisionListener(member, _detector);
                    }
                }
            }

            _initialized = true;
            Debug.Log($"[RMCollision] Initialized with {_segments.Length} segments. " +
                $"Calibrating for {_config.CalibrationDurationSec}s...");
        }

        /// <summary>
        /// 每帧调用。推进校准器和评分衰减。
        /// </summary>
        public void Tick(float deltaTime)
        {
            if (!_initialized) return;
            _detector?.Tick(deltaTime);
        }

        /// <summary>
        /// 获取所有碰撞体的包围盒（供 Gizmo 绘制）。
        /// </summary>
        public RMBounds[] GetAllColliderBounds()
        {
            if (_colliderManager == null) return new RMBounds[0];
            return _colliderManager.GetAllBounds();
        }

        /// <summary>
        /// 绘制碰撞体线框 Gizmo（AABB 包围盒）。
        /// 由 RobotArmBehaviour.OnDrawGizmos 调用。
        /// </summary>
        /// <param name="visualizer">引擎可视化接口。</param>
        public void DrawColliderGizmos(IEngineVisualizer visualizer)
        {
            if (!_initialized || !_config.ShowCollidersGizmo || visualizer == null) return;

            var bounds = GetAllColliderBounds();
            var gizmoColor = new RMColor(0f, 1f, 1f, 0.3f); // 半透明青色

            foreach (var b in bounds)
            {
                if (!b.IsValid) continue;
                DrawWireBox(visualizer, b.Center, b.Size, gizmoColor);
            }

            // 高亮活跃碰撞对
            if (_detector != null)
            {
                var activeColor = new RMColor(1f, 0f, 0f, 0.5f); // 半透明红色
                foreach (var kvp in _detector.AccumulatedScores)
                {
                    if (kvp.Value < 0.01f) continue;
                    if (_segments == null) continue;
                    if (kvp.Key.SegmentA < _segments.Length && kvp.Key.SegmentB < _segments.Length)
                    {
                        var posA = _segments[kvp.Key.SegmentA].RootJoint.WorldPosition;
                        var posB = _segments[kvp.Key.SegmentB].RootJoint.WorldPosition;
                        visualizer.DrawLine(posA, posB, activeColor);
                    }
                }
            }
        }

        private static void DrawWireBox(IEngineVisualizer vis, RMVector3 center, RMVector3 size, RMColor color)
        {
            float hx = size.X * 0.5f;
            float hy = size.Y * 0.5f;
            float hz = size.Z * 0.5f;

            var v0 = new RMVector3(center.X - hx, center.Y - hy, center.Z - hz);
            var v1 = new RMVector3(center.X + hx, center.Y - hy, center.Z - hz);
            var v2 = new RMVector3(center.X + hx, center.Y + hy, center.Z - hz);
            var v3 = new RMVector3(center.X - hx, center.Y + hy, center.Z - hz);
            var v4 = new RMVector3(center.X - hx, center.Y - hy, center.Z + hz);
            var v5 = new RMVector3(center.X + hx, center.Y - hy, center.Z + hz);
            var v6 = new RMVector3(center.X + hx, center.Y + hy, center.Z + hz);
            var v7 = new RMVector3(center.X - hx, center.Y + hy, center.Z + hz);

            vis.DrawLine(v0, v1, color); vis.DrawLine(v1, v2, color);
            vis.DrawLine(v2, v3, color); vis.DrawLine(v3, v0, color);
            vis.DrawLine(v4, v5, color); vis.DrawLine(v5, v6, color);
            vis.DrawLine(v6, v7, color); vis.DrawLine(v7, v4, color);
            vis.DrawLine(v0, v4, color); vis.DrawLine(v1, v5, color);
            vis.DrawLine(v2, v6, color); vis.DrawLine(v3, v7, color);
        }

        // ===== 生命周期 =====

        /// <summary>
        /// 清理并释放资源。在 OnDestroy 中调用。
        /// </summary>
        public void Dispose()
        {
            if (!_initialized) return;

            // 注销碰撞监听
            if (_segments != null && _physics != null)
            {
                foreach (var segment in _segments)
                {
                    if (segment.Members == null) continue;
                    foreach (var member in segment.Members)
                    {
                        if (member != null && member.IsValid)
                            _physics.UnregisterCollisionListener(member);
                    }
                }
            }

            _detector?.Dispose();
            _colliderManager?.RemoveAllColliders();
            _initialized = false;
        }

        private void OnCollisionAlert(CollisionPairKey pair, float score, string eventType,
            string colliderNameA, string colliderNameB)
        {
            if (eventType == "Enter")
            {
                string labelA = pair.SegmentA == _externalSegmentIndex ? "外部" : $"轴{pair.SegmentA}";
                string labelB = pair.SegmentB == _externalSegmentIndex ? "外部" : $"轴{pair.SegmentB}";
                Debug.LogWarning($"[RMCollision] 碰撞检测: {labelA}的'{colliderNameA}' <-> " +
                    $"{labelB}的'{colliderNameB}', Score: {score:F3}");
            }
        }
    }
}
