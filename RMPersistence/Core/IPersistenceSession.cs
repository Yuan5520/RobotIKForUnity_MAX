using System;

namespace RMPersistence
{
    /// <summary>
    /// 持久化会话接口。
    /// 每次录制/检测会话对应一个 Session 实例，负责序列化写入和关闭。
    /// </summary>
    public interface IPersistenceSession : IDisposable
    {
        /// <summary>
        /// 将一条记录序列化为 JSON 并写入队列。
        /// </summary>
        void Write(object record);

        /// <summary>
        /// 关闭会话，刷新剩余数据并结束文件写入。
        /// </summary>
        void Close();

        /// <summary>
        /// 会话是否已关闭。
        /// </summary>
        bool IsClosed { get; }

        /// <summary>
        /// 输出文件路径。
        /// </summary>
        string FilePath { get; }
    }
}
