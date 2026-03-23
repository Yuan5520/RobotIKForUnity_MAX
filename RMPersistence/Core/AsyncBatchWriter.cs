using System;
using System.Collections.Concurrent;
using System.IO;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace RMPersistence
{
    /// <summary>
    /// 异步批量写入器。
    /// 使用 ConcurrentQueue + 后台 Task 实现高性能异步 JSON 写入。
    /// </summary>
    public class AsyncBatchWriter
    {
        private readonly string _filePath;
        private readonly PersistenceConfig _config;
        private readonly ConcurrentQueue<string> _queue;
        private readonly CancellationTokenSource _cts;
        private readonly Task _consumerTask;

        private volatile bool _isRunning;
        private int _droppedCount;
        private bool _firstEntry;

        public AsyncBatchWriter(string filePath, PersistenceConfig config)
        {
            _filePath = filePath;
            _config = config ?? new PersistenceConfig();
            _queue = new ConcurrentQueue<string>();
            _cts = new CancellationTokenSource();
            _isRunning = true;
            _firstEntry = true;
            _droppedCount = 0;

            // 写入 JSON 数组开头
            File.WriteAllText(_filePath, "[\n", Encoding.UTF8);

            _consumerTask = Task.Run(() => ConsumerLoop(_cts.Token));
        }

        /// <summary>
        /// 线程安全入队。超过 MaxQueueSize 时丢弃最旧数据。
        /// </summary>
        public void Enqueue(string jsonLine)
        {
            if (!_isRunning) return;

            // 溢出保护：丢弃最旧数据
            while (_queue.Count >= _config.MaxQueueSize)
            {
                _queue.TryDequeue(out _);
                Interlocked.Increment(ref _droppedCount);
            }

            _queue.Enqueue(jsonLine);
        }

        /// <summary>
        /// 强制刷新队列中所有数据到文件。
        /// </summary>
        public async Task FlushAsync()
        {
            await Task.Run(() => FlushInternal());
        }

        /// <summary>
        /// 停止后台循环并执行最终刷新。
        /// </summary>
        public async Task CloseAsync()
        {
            _isRunning = false;
            _cts.Cancel();

            try
            {
                await _consumerTask;
            }
            catch (OperationCanceledException)
            {
                // 正常取消
            }

            // 最终刷新
            FlushInternal();

            // 写入溢出警告（如有）
            if (_droppedCount > 0)
            {
                string warning = $",\n  {{\"_warning\": \"overflow_dropped\", \"count\": {_droppedCount}}}";
                File.AppendAllText(_filePath, warning, Encoding.UTF8);
            }

            // 关闭 JSON 数组
            File.AppendAllText(_filePath, "\n]\n", Encoding.UTF8);
        }

        /// <summary>
        /// 同步关闭（OnDestroy 中使用）。
        /// </summary>
        public void CloseSync()
        {
            _isRunning = false;
            _cts.Cancel();

            try
            {
                _consumerTask.Wait(3000); // 最多等待 3 秒
            }
            catch
            {
                // 超时或异常，继续执行最终刷新
            }

            FlushInternal();

            if (_droppedCount > 0)
            {
                string warning = $",\n  {{\"_warning\": \"overflow_dropped\", \"count\": {_droppedCount}}}";
                File.AppendAllText(_filePath, warning, Encoding.UTF8);
            }

            File.AppendAllText(_filePath, "\n]\n", Encoding.UTF8);
        }

        private async Task ConsumerLoop(CancellationToken token)
        {
            var lastFlushTime = DateTime.UtcNow;

            while (!token.IsCancellationRequested)
            {
                bool shouldFlush = _queue.Count >= _config.BatchSize ||
                    (DateTime.UtcNow - lastFlushTime).TotalMilliseconds >= _config.FlushIntervalMs;

                if (shouldFlush && _queue.Count > 0)
                {
                    FlushInternal();
                    lastFlushTime = DateTime.UtcNow;
                }

                try
                {
                    await Task.Delay(50, token);
                }
                catch (OperationCanceledException)
                {
                    break;
                }
            }
        }

        private void FlushInternal()
        {
            var sb = new StringBuilder();
            int count = 0;

            while (_queue.TryDequeue(out string line))
            {
                if (_firstEntry)
                {
                    sb.Append("  ");
                    _firstEntry = false;
                }
                else
                {
                    sb.Append(",\n  ");
                }
                sb.Append(line);
                count++;
            }

            if (count > 0)
            {
                File.AppendAllText(_filePath, sb.ToString(), Encoding.UTF8);
            }
        }
    }
}
