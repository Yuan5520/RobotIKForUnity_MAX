namespace IEngine
{
    /// <summary>
    /// 引擎适配器工厂接口。
    /// 负责创建各类引擎抽象层的具体实现实例。
    /// </summary>
    public interface IEngineAdapterFactory
    {
        /// <summary>
        /// 将引擎原生对象包装为 IEngineObject。
        /// </summary>
        IEngineObject WrapObject(object nativeObject);

        /// <summary>
        /// 创建被动可视化绘制器实例。
        /// </summary>
        IEngineVisualizer CreateVisualizer();

        /// <summary>
        /// 创建交互式控制柄实例。
        /// 编辑器返回可用实例，运行时返回空实现（NullHandle）。
        /// </summary>
        IEngineHandle CreateHandle();

        /// <summary>
        /// 创建输入抽象实例。
        /// </summary>
        IEngineInput CreateInput();

        /// <summary>
        /// 创建物理引擎抽象实例。
        /// </summary>
        IEnginePhysics CreatePhysics();
    }
}
