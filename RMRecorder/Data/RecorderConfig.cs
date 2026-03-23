using System;

namespace RMRecorder
{
    /// <summary>
    /// 数据记录器配置。
    /// </summary>
    [Serializable]
    public class RecorderConfig
    {
        /// <summary>采样频率（Hz）。</summary>
        public float SamplingFrequencyHz = 30f;

        /// <summary>是否启用滑动窗口数据平滑。</summary>
        public bool EnableSmoothing = true;

        /// <summary>滑动窗口大小（帧数）。</summary>
        public int SlidingWindowSize = 5;

        /// <summary>
        /// 异常阈值（弧度）。相邻两帧关节角度跳变超过此值视为异常，将被丢弃。
        /// 默认 30 度 ≈ 0.5236 弧度。
        /// </summary>
        public float AnomalyThresholdRad = 0.5236f;

        /// <summary>
        /// 插值最大间隔（弧度）。相邻两帧角度差超过此值时插入中间值。
        /// 默认 15 度 ≈ 0.2618 弧度。
        /// </summary>
        public float InterpolationMaxGapRad = 0.2618f;

        /// <summary>采样间隔（秒），由 SamplingFrequencyHz 计算。</summary>
        public float SamplingInterval => SamplingFrequencyHz > 0f ? 1f / SamplingFrequencyHz : 0.033f;
    }
}
