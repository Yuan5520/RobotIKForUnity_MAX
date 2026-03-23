using System;

namespace RMPersistence
{
    /// <summary>
    /// 持久化配置。
    /// </summary>
    [Serializable]
    public class PersistenceConfig
    {
        /// <summary>批次大小：每累积 N 条记录刷新一次。</summary>
        public int BatchSize = 100;

        /// <summary>最大刷新间隔（毫秒）。</summary>
        public int FlushIntervalMs = 1000;

        /// <summary>队列最大容量（防溢出）。</summary>
        public int MaxQueueSize = 10000;
    }
}
