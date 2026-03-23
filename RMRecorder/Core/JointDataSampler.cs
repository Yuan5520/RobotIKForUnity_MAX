namespace RMRecorder
{
    /// <summary>
    /// 关节数据采样器。
    /// 根据配置的采样频率控制采样时机，避免每帧都记录。
    /// </summary>
    public class JointDataSampler
    {
        private readonly RecorderConfig _config;
        private float _timeSinceLastSample;

        public JointDataSampler(RecorderConfig config)
        {
            _config = config;
            _timeSinceLastSample = 0f;
        }

        /// <summary>
        /// 判断当前帧是否应该采样。
        /// </summary>
        /// <param name="deltaTime">自上一帧以来的时间（秒）。</param>
        /// <returns>如果应该采样返回 true。</returns>
        public bool ShouldSample(float deltaTime)
        {
            _timeSinceLastSample += deltaTime;
            if (_timeSinceLastSample >= _config.SamplingInterval)
            {
                _timeSinceLastSample -= _config.SamplingInterval;
                // 防止长时间暂停后连续触发多次
                if (_timeSinceLastSample > _config.SamplingInterval)
                    _timeSinceLastSample = 0f;
                return true;
            }
            return false;
        }

        /// <summary>
        /// 重置采样计时器。
        /// </summary>
        public void Reset()
        {
            _timeSinceLastSample = 0f;
        }
    }
}
