using System.Collections.Generic;

namespace RMCollision
{
    /// <summary>
    /// 静态干涉校准器。
    /// 在初始化阶段记录机械臂静态状态下已存在的碰撞对（相邻段的几何接触），
    /// 校准完成后将这些碰撞对排除在告警队列之外。
    /// </summary>
    public class StaticInterferenceCalibrator
    {
        private readonly float _calibrationDuration;
        private float _elapsedTime;
        private bool _isCalibrating;
        private bool _calibrationDone;

        private readonly HashSet<CollisionPairKey> _baselinePairs;

        /// <summary>是否正在校准中。</summary>
        public bool IsCalibrating => _isCalibrating;

        /// <summary>校准是否已完成。</summary>
        public bool IsCalibrated => _calibrationDone;

        /// <summary>基线碰撞对数量。</summary>
        public int BaselineCount => _baselinePairs.Count;

        public StaticInterferenceCalibrator(float calibrationDurationSec)
        {
            _calibrationDuration = calibrationDurationSec;
            _elapsedTime = 0f;
            _isCalibrating = true;
            _calibrationDone = false;
            _baselinePairs = new HashSet<CollisionPairKey>();
        }

        /// <summary>
        /// 校准阶段推进时间。到达校准时长后自动结束校准。
        /// </summary>
        /// <param name="deltaTime">帧间隔时间（秒）。</param>
        public void Tick(float deltaTime)
        {
            if (!_isCalibrating) return;

            _elapsedTime += deltaTime;
            if (_elapsedTime >= _calibrationDuration)
            {
                _isCalibrating = false;
                _calibrationDone = true;
                UnityEngine.Debug.Log($"[RMCollision] Static interference calibration complete. " +
                    $"Baseline pairs: {_baselinePairs.Count}");
            }
        }

        /// <summary>
        /// 校准阶段记录一个碰撞对到基线。
        /// </summary>
        public void RecordBaselinePair(CollisionPairKey pair)
        {
            if (!_isCalibrating) return;
            _baselinePairs.Add(pair);
        }

        /// <summary>
        /// 判断碰撞对是否在基线排除列表中。
        /// </summary>
        public bool IsBaselinePair(CollisionPairKey pair)
        {
            return _baselinePairs.Contains(pair);
        }

        /// <summary>
        /// 重置校准器。
        /// </summary>
        public void Reset()
        {
            _elapsedTime = 0f;
            _isCalibrating = true;
            _calibrationDone = false;
            _baselinePairs.Clear();
        }
    }
}
