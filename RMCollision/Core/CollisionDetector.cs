using System;
using System.Collections.Generic;
using IEngine;
using RMPersistence;
using UnityEngine;

namespace RMCollision
{
    /// <summary>
    /// 碰撞检测器。
    /// 实现 ICollisionListener，接收引擎层碰撞事件，
    /// 映射到关节段、评分、排除基线碰撞对、持久化记录。
    /// Stay 事件采用指数退避写入策略以避免日志膨胀。
    /// </summary>
    public class CollisionDetector : ICollisionListener
    {
        private readonly CollisionConfig _config;
        private readonly Dictionary<string, int> _objectToSegment;
        private readonly StaticInterferenceCalibrator _calibrator;
        private readonly Dictionary<CollisionPairKey, float> _accumulatedScores;
        private IPersistenceSession _persistence;

        // Stay 指数退避：记录每个碰撞对的下次允许写入时间和当前间隔
        private readonly Dictionary<CollisionPairKey, StayThrottleState> _stayThrottle;

        /// <summary>Stay 事件的初始写入间隔（秒）。</summary>
        private const float StayInitialInterval = 1f;
        /// <summary>Stay 事件间隔开始指数增长的累计持续时间阈值（秒）。</summary>
        private const float StayExpGrowthThreshold = 10f;
        /// <summary>Stay 事件间隔的指数增长倍率。</summary>
        private const float StayExpGrowthFactor = 2f;

        /// <summary>当前各碰撞对的累积评分（只读快照）。</summary>
        public IReadOnlyDictionary<CollisionPairKey, float> AccumulatedScores => _accumulatedScores;

        /// <summary>当检测到非基线碰撞时触发。参数：(碰撞对, 评分, 事件类型, 碰撞体A名称, 碰撞体B名称)。</summary>
        public event Action<CollisionPairKey, float, string, string, string> OnCollisionDetected;

        public CollisionDetector(CollisionConfig config,
                                  Dictionary<string, int> objectToSegment,
                                  StaticInterferenceCalibrator calibrator,
                                  IPersistenceSession persistence)
        {
            _config = config;
            _objectToSegment = objectToSegment;
            _calibrator = calibrator;
            _persistence = persistence;
            _accumulatedScores = new Dictionary<CollisionPairKey, float>();
            _stayThrottle = new Dictionary<CollisionPairKey, StayThrottleState>();
        }

        /// <summary>
        /// 每帧更新：衰减累积评分、推进校准器。
        /// </summary>
        public void Tick(float deltaTime)
        {
            _calibrator.Tick(deltaTime);

            // 衰减累积评分
            var keys = new List<CollisionPairKey>(_accumulatedScores.Keys);
            foreach (var key in keys)
            {
                float decayed = CollisionScorer.ApplyDecay(_accumulatedScores[key], 0f,
                    _config.ScoreDecayRate, deltaTime);
                if (decayed < 0.001f)
                    _accumulatedScores.Remove(key);
                else
                    _accumulatedScores[key] = decayed;
            }
        }

        public void OnCollisionEnter(RMCollisionEventData data)
        {
            ProcessCollision(data, "Enter");
        }

        public void OnCollisionStay(RMCollisionEventData data)
        {
            ProcessCollision(data, "Stay");
        }

        public void OnCollisionExit(RMCollisionEventData data)
        {
            ProcessCollision(data, "Exit");
        }

        private void ProcessCollision(RMCollisionEventData data, string eventType)
        {
            // 映射到段索引
            if (data.Self == null || data.Other == null) return;

            string selfName = data.Self.Name;
            string otherName = data.Other.Name;

            if (!_objectToSegment.TryGetValue(selfName, out int segA)) return;
            if (!_objectToSegment.TryGetValue(otherName, out int segB)) return;

            // 同段碰撞忽略
            if (segA == segB) return;

            var pairKey = new CollisionPairKey(segA, segB);

            // 校准阶段：记录到基线
            if (_calibrator.IsCalibrating)
            {
                _calibrator.RecordBaselinePair(pairKey);
                return;
            }

            // 校准完成：检查是否为基线碰撞对
            if (_calibrator.IsCalibrated && _calibrator.IsBaselinePair(pairKey))
                return;

            // 计算评分
            float score = CollisionScorer.CalculateScore(data.RelativeVelocity, data.ContactArea);

            // 确保碰撞体名称与段顺序一致（CollisionPairKey 保证 SegmentA < SegmentB）
            string colliderNameA, colliderNameB;
            if (segA <= segB)
            {
                colliderNameA = selfName;
                colliderNameB = otherName;
            }
            else
            {
                colliderNameA = otherName;
                colliderNameB = selfName;
            }

            // 更新累积评分（Exit 事件不累加）
            if (eventType != "Exit")
            {
                if (!_accumulatedScores.ContainsKey(pairKey))
                    _accumulatedScores[pairKey] = 0f;
                _accumulatedScores[pairKey] += score;
            }

            // 触发事件回调（Enter/Stay/Exit 均触发，由上层决定如何处理）
            OnCollisionDetected?.Invoke(pairKey, score, eventType, colliderNameA, colliderNameB);

            // ---- 持久化写入（Stay 采用指数退避） ----
            if (!_config.EnablePersistence || _persistence == null || _persistence.IsClosed)
                return;

            if (eventType == "Enter")
            {
                // Enter：实时写入，并初始化 Stay 节流状态
                WritePersistence(pairKey, score, eventType, colliderNameA, colliderNameB);
                _stayThrottle[pairKey] = new StayThrottleState
                {
                    StayStartTime = Time.time,
                    NextWriteTime = Time.time + StayInitialInterval,
                    CurrentInterval = StayInitialInterval
                };
            }
            else if (eventType == "Exit")
            {
                // Exit：实时写入，清除节流状态
                WritePersistence(pairKey, score, eventType, colliderNameA, colliderNameB);
                _stayThrottle.Remove(pairKey);
            }
            else // Stay
            {
                // Stay：指数退避写入
                float now = Time.time;

                if (!_stayThrottle.TryGetValue(pairKey, out var state))
                {
                    // 可能没经过 Enter 就收到 Stay（罕见），初始化
                    state = new StayThrottleState
                    {
                        StayStartTime = now,
                        NextWriteTime = now + StayInitialInterval,
                        CurrentInterval = StayInitialInterval
                    };
                    _stayThrottle[pairKey] = state;
                }

                if (now >= state.NextWriteTime)
                {
                    WritePersistence(pairKey, score, eventType, colliderNameA, colliderNameB);

                    // 持续超过阈值后，间隔指数增长
                    float stayDuration = now - state.StayStartTime;
                    float newInterval = state.CurrentInterval;
                    if (stayDuration >= StayExpGrowthThreshold)
                        newInterval = state.CurrentInterval * StayExpGrowthFactor;

                    state.NextWriteTime = now + newInterval;
                    state.CurrentInterval = newInterval;
                    _stayThrottle[pairKey] = state;
                }
            }
        }

        private void WritePersistence(CollisionPairKey pairKey, float score,
            string eventType, string colliderNameA, string colliderNameB)
        {
            var evt = new CollisionEvent
            {
                Timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"),
                SegmentA = pairKey.SegmentA,
                SegmentB = pairKey.SegmentB,
                ColliderNameA = colliderNameA,
                ColliderNameB = colliderNameB,
                Score = score,
                EventType = eventType
            };
            _persistence.Write(evt);
        }

        /// <summary>
        /// 关闭持久化会话。
        /// </summary>
        public void Dispose()
        {
            _persistence?.Close();
            _persistence = null;
        }

        /// <summary>Stay 事件节流状态。</summary>
        private struct StayThrottleState
        {
            /// <summary>Stay 开始的时间（用于判断是否超过指数增长阈值）。</summary>
            public float StayStartTime;
            /// <summary>下次允许写入的时间。</summary>
            public float NextWriteTime;
            /// <summary>当前写入间隔（秒）。</summary>
            public float CurrentInterval;
        }
    }
}
