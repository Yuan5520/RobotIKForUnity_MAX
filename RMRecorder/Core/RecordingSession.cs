using System;
using RMPersistence;

namespace RMRecorder
{
    /// <summary>
    /// 单次录制会话。
    /// 管理采样器、平滑器、持久化会话的生命周期。
    /// </summary>
    public class RecordingSession : IDisposable
    {
        private readonly RecorderConfig _config;
        private readonly int _dof;
        private readonly JointDataSampler _sampler;
        private readonly JointDataSmoother _smoother;
        private readonly IPersistenceSession _persistence;
        private float _elapsedTime;
        private bool _isActive;
        private int _recordCount;

        /// <summary>录制是否正在进行中。</summary>
        public bool IsActive => _isActive;

        /// <summary>已录制的帧数。</summary>
        public int RecordCount => _recordCount;

        /// <summary>录制持续时间（秒）。</summary>
        public float ElapsedTime => _elapsedTime;

        /// <summary>输出文件路径。</summary>
        public string FilePath => _persistence?.FilePath;

        public RecordingSession(int dof, RecorderConfig config, PersistenceConfig persistenceConfig = null)
        {
            _dof = dof;
            _config = config ?? new RecorderConfig();
            _sampler = new JointDataSampler(_config);
            _smoother = _config.EnableSmoothing ? new JointDataSmoother(dof, _config) : null;
            _persistence = PersistenceManager.CreateSession("RMRecorder", persistenceConfig);
            _elapsedTime = 0f;
            _isActive = true;
            _recordCount = 0;
        }

        /// <summary>
        /// 每帧调用，传入当前关节角度。
        /// 内部根据采样频率和平滑配置决定是否记录。
        /// </summary>
        /// <param name="deltaTime">帧间隔时间（秒）。</param>
        /// <param name="jointAnglesRad">当前各关节角度（弧度）。</param>
        public void Tick(float deltaTime, float[] jointAnglesRad)
        {
            if (!_isActive) return;

            _elapsedTime += deltaTime;

            if (!_sampler.ShouldSample(deltaTime))
                return;

            // 复制原始数据
            var rawAngles = new float[_dof];
            int copyLen = Math.Min(jointAnglesRad.Length, _dof);
            Array.Copy(jointAnglesRad, rawAngles, copyLen);

            // 平滑处理
            float[] smoothedAngles = rawAngles;
            if (_smoother != null)
            {
                var result = _smoother.Process(rawAngles);
                if (result == null)
                    return; // 异常帧，跳过
                smoothedAngles = result;
            }

            // 构造记录
            var record = new JointDataRecord
            {
                Timestamp = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss.fff"),
                JointAnglesRad = rawAngles,
                SmoothedAnglesRad = smoothedAngles
            };

            _persistence.Write(record);
            _recordCount++;
        }

        /// <summary>
        /// 停止录制并关闭持久化会话。
        /// </summary>
        public void Stop()
        {
            if (!_isActive) return;
            _isActive = false;
            _persistence?.Close();
        }

        public void Dispose()
        {
            Stop();
        }
    }
}
