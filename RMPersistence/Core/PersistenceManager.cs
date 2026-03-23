using UnityEngine;

namespace RMPersistence
{
    /// <summary>
    /// 持久化管理器。
    /// 提供创建持久化会话的工厂方法。
    /// </summary>
    public static class PersistenceManager
    {
        /// <summary>
        /// 创建一个新的持久化会话。
        /// </summary>
        /// <param name="suiteName">套件名称，用于确定存储目录和文件名前缀。</param>
        /// <param name="config">持久化配置。传 null 则使用默认值。</param>
        /// <returns>可用于写入数据的 IPersistenceSession 实例。</returns>
        public static IPersistenceSession CreateSession(string suiteName, PersistenceConfig config = null)
        {
            var cfg = config ?? new PersistenceConfig();
            string filePath = FilePathResolver.GenerateFilePath(suiteName);
            Debug.Log($"[RMPersistence] Session created: {filePath}");
            return new PersistenceSession(filePath, cfg);
        }
    }

    /// <summary>
    /// IPersistenceSession 的默认实现。
    /// 内部使用 AsyncBatchWriter 进行异步批量写入。
    /// </summary>
    internal class PersistenceSession : IPersistenceSession
    {
        private readonly AsyncBatchWriter _writer;
        private readonly string _filePath;
        private volatile bool _isClosed;

        public bool IsClosed => _isClosed;
        public string FilePath => _filePath;

        public PersistenceSession(string filePath, PersistenceConfig config)
        {
            _filePath = filePath;
            _isClosed = false;
            _writer = new AsyncBatchWriter(filePath, config);
        }

        public void Write(object record)
        {
            if (_isClosed)
            {
                Debug.LogWarning("[RMPersistence] Attempted to write to a closed session.");
                return;
            }

            string json = JsonUtility.ToJson(record);
            _writer.Enqueue(json);
        }

        public void Close()
        {
            if (_isClosed) return;
            _isClosed = true;
            _writer.CloseSync();
            Debug.Log($"[RMPersistence] Session closed: {_filePath}");
        }

        public void Dispose()
        {
            Close();
        }
    }
}
