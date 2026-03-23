using System;
using System.Collections.Generic;

namespace RMRecorder
{
    /// <summary>
    /// 滑动窗口数据平滑器。
    /// 三阶段处理管线：
    ///   1. 异常检测：相邻帧跳变超过阈值的数据点被丢弃
    ///   2. 间隔插值：相邻帧角度差超过插值阈值时插入中间帧
    ///   3. 移动平均：对窗口内的数据进行均值平滑
    /// </summary>
    public class JointDataSmoother
    {
        private readonly RecorderConfig _config;
        private readonly int _dof;

        // 滑动窗口：每个关节维护独立的样本缓冲
        private readonly Queue<float>[] _windows;
        private float[] _lastValidAngles;
        private bool _hasLastValid;

        public JointDataSmoother(int dof, RecorderConfig config)
        {
            _dof = dof;
            _config = config;
            _windows = new Queue<float>[dof];
            for (int i = 0; i < dof; i++)
                _windows[i] = new Queue<float>();
            _lastValidAngles = new float[dof];
            _hasLastValid = false;
        }

        /// <summary>
        /// 处理一帧原始关节角度，返回平滑后的角度。
        /// 如果该帧被判定为异常，返回 null（表示应跳过此帧）。
        /// </summary>
        public float[] Process(float[] rawAngles)
        {
            if (rawAngles == null || rawAngles.Length != _dof)
                return null;

            // === 阶段 1：异常检测 ===
            if (_hasLastValid)
            {
                for (int i = 0; i < _dof; i++)
                {
                    float diff = Math.Abs(rawAngles[i] - _lastValidAngles[i]);
                    if (diff > _config.AnomalyThresholdRad)
                    {
                        // 整帧被判定为异常，丢弃
                        return null;
                    }
                }
            }

            // === 阶段 2：间隔插值 ===
            // 如果某关节的角度跳变超过插值阈值，先插入中间帧到窗口
            if (_hasLastValid)
            {
                for (int i = 0; i < _dof; i++)
                {
                    float gap = rawAngles[i] - _lastValidAngles[i];
                    float absGap = Math.Abs(gap);

                    if (absGap > _config.InterpolationMaxGapRad)
                    {
                        // 计算需要插入多少中间值
                        int steps = (int)Math.Ceiling(absGap / _config.InterpolationMaxGapRad);
                        for (int s = 1; s < steps; s++)
                        {
                            float t = (float)s / steps;
                            float interpolated = _lastValidAngles[i] + gap * t;
                            EnqueueToWindow(i, interpolated);
                        }
                    }
                }
            }

            // 将当前帧数据入窗口
            for (int i = 0; i < _dof; i++)
                EnqueueToWindow(i, rawAngles[i]);

            // 更新上一帧有效数据
            Array.Copy(rawAngles, _lastValidAngles, _dof);
            _hasLastValid = true;

            // === 阶段 3：移动平均 ===
            var smoothed = new float[_dof];
            for (int i = 0; i < _dof; i++)
            {
                float sum = 0f;
                int count = 0;
                foreach (float val in _windows[i])
                {
                    sum += val;
                    count++;
                }
                smoothed[i] = count > 0 ? sum / count : rawAngles[i];
            }

            return smoothed;
        }

        /// <summary>
        /// 重置平滑器状态。
        /// </summary>
        public void Reset()
        {
            for (int i = 0; i < _dof; i++)
                _windows[i].Clear();
            _hasLastValid = false;
        }

        private void EnqueueToWindow(int jointIndex, float value)
        {
            var window = _windows[jointIndex];
            window.Enqueue(value);
            while (window.Count > _config.SlidingWindowSize)
                window.Dequeue();
        }
    }
}
