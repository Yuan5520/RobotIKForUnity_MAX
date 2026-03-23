using System;

namespace RMRecorder
{
    /// <summary>
    /// 单帧关节数据记录。
    /// 由 JsonUtility 序列化，字段必须为 public。
    /// </summary>
    [Serializable]
    public class JointDataRecord
    {
        /// <summary>真实时间戳，格式：yyyy-MM-dd HH:mm:ss.fff</summary>
        public string Timestamp;

        /// <summary>各关节角度（弧度）。</summary>
        public float[] JointAnglesRad;

        /// <summary>平滑后的关节角度（弧度），未启用平滑时与原始值相同。</summary>
        public float[] SmoothedAnglesRad;
    }
}
