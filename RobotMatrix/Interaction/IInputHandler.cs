namespace RobotMatrix.Interaction
{
    /// <summary>
    /// 输入处理器接口 — 所有输入源（键盘、鼠标、编辑器手柄）的统一抽象。
    /// 每帧调用 Update()，通过回调或轮询方式产生 IControlCommand。
    /// </summary>
    public interface IInputHandler
    {
        /// <summary>
        /// 处理器是否启用。禁用时 Update() 不产生命令。
        /// </summary>
        bool Enabled { get; set; }

        /// <summary>
        /// 每帧调用，检测输入并返回产生的命令。
        /// 若本帧无输入，返回 null。
        /// </summary>
        IControlCommand Update();
    }
}
