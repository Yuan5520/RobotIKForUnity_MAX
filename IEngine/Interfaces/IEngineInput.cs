using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 输入抽象接口，封装引擎原生输入系统（鼠标/键盘）。
    /// </summary>
    public interface IEngineInput
    {
        /// <summary>
        /// 将屏幕坐标转换为指定深度处的世界坐标。
        /// </summary>
        RMVector3 GetMouseWorldPosition(RMVector3 screenPos, float depth);

        /// <summary>
        /// 指定鼠标按钮是否处于按下状态（持续检测）。
        /// </summary>
        bool GetMouseButton(int button);

        /// <summary>
        /// 指定鼠标按钮是否在本帧按下（单次触发）。
        /// </summary>
        bool GetMouseButtonDown(int button);

        /// <summary>
        /// 指定键是否处于按下状态（持续检测）。
        /// </summary>
        bool GetKey(string keyName);

        /// <summary>
        /// 指定键是否在本帧按下（单次触发）。
        /// </summary>
        bool GetKeyDown(string keyName);

        /// <summary>
        /// 获取虚拟轴值（如 "Horizontal"、"Vertical"、"Mouse X"）。
        /// </summary>
        float GetAxis(string axisName);
    }
}
