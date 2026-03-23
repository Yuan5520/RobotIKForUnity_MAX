using UnityEngine;
using RMPersistence;

namespace RMRecorder
{
    /// <summary>
    /// 数据记录套件控制器。
    /// 提供手动 Start/Stop 录制 API，由外层（如 RobotArmBehaviour）每帧驱动。
    /// </summary>
    public class RecorderSuiteController
    {
        private RecorderConfig _config;
        private PersistenceConfig _persistenceConfig;
        private RecordingSession _currentSession;
        private int _dof;

        /// <summary>当前录制会话（null 表示未在录制）。</summary>
        public RecordingSession CurrentSession => _currentSession;

        /// <summary>是否正在录制。</summary>
        public bool IsRecording => _currentSession != null && _currentSession.IsActive;

        public RecorderSuiteController(int dof, RecorderConfig config, PersistenceConfig persistenceConfig = null)
        {
            _dof = dof;
            _config = config ?? new RecorderConfig();
            _persistenceConfig = persistenceConfig;
        }

        /// <summary>
        /// 开始录制。若当前已在录制，先停止旧会话。
        /// </summary>
        public void StartRecording()
        {
            if (IsRecording)
                StopRecording();

            _currentSession = new RecordingSession(_dof, _config, _persistenceConfig);
            Debug.Log($"[RMRecorder] Recording started. File: {_currentSession.FilePath}");
        }

        /// <summary>
        /// 停止录制。
        /// </summary>
        public void StopRecording()
        {
            if (_currentSession == null) return;

            _currentSession.Stop();
            Debug.Log($"[RMRecorder] Recording stopped. Frames: {_currentSession.RecordCount}, " +
                      $"Duration: {_currentSession.ElapsedTime:F2}s, File: {_currentSession.FilePath}");
            _currentSession = null;
        }

        /// <summary>
        /// 每帧调用。如果正在录制，传入当前关节角度。
        /// </summary>
        /// <param name="deltaTime">帧间隔时间（秒）。</param>
        /// <param name="jointAnglesRad">当前各关节角度（弧度）。</param>
        public void Tick(float deltaTime, float[] jointAnglesRad)
        {
            if (!IsRecording) return;
            _currentSession.Tick(deltaTime, jointAnglesRad);
        }

        /// <summary>
        /// 运行时更新配置（下次 StartRecording 生效）。
        /// </summary>
        public void UpdateConfig(RecorderConfig config, PersistenceConfig persistenceConfig = null)
        {
            _config = config ?? new RecorderConfig();
            if (persistenceConfig != null)
                _persistenceConfig = persistenceConfig;
        }

        /// <summary>
        /// 更新自由度（当机械臂配置变更时调用）。
        /// </summary>
        public void UpdateDOF(int dof)
        {
            _dof = dof;
        }

        /// <summary>
        /// 清理资源。在 OnDestroy 中调用。
        /// </summary>
        public void Dispose()
        {
            StopRecording();
        }
    }
}
