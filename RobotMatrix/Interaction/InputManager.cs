using System.Collections.Generic;

namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 输入管理器 — 管理多个 IInputHandler，按优先级轮询产生命令。
    /// 每帧仅执行第一个非 null 命令（优先级由注册顺序决定）。
    /// </summary>
    public class InputManager
    {
        private readonly List<IInputHandler> _handlers = new List<IInputHandler>();
        private readonly CommandDispatcher _dispatcher;

        public InputManager(CommandDispatcher dispatcher)
        {
            _dispatcher = dispatcher;
        }

        /// <summary>
        /// 注册输入处理器。先注册的优先级更高。
        /// </summary>
        public void Register(IInputHandler handler)
        {
            if (handler != null && !_handlers.Contains(handler))
                _handlers.Add(handler);
        }

        /// <summary>
        /// 移除输入处理器。
        /// </summary>
        public void Unregister(IInputHandler handler)
        {
            _handlers.Remove(handler);
        }

        /// <summary>
        /// 已注册的处理器数量。
        /// </summary>
        public int HandlerCount => _handlers.Count;

        /// <summary>
        /// 每帧调用：轮询所有处理器，将第一个有效命令分发给控制器。
        /// </summary>
        /// <returns>本帧是否成功执行了命令。</returns>
        public bool Tick()
        {
            for (int i = 0; i < _handlers.Count; i++)
            {
                var cmd = _handlers[i].Update();
                if (cmd != null)
                    return _dispatcher.Dispatch(cmd);
            }
            return false;
        }
    }
}
