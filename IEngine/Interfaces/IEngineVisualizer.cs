using RobotMatrix.Math;

namespace IEngine
{
    /// <summary>
    /// 被动可视化接口，负责绘制调试/辅助图形。
    /// Unity 实现：编辑器下使用 Gizmos API，运行时使用 GL/Debug.DrawLine。
    /// </summary>
    public interface IEngineVisualizer
    {
        void DrawLine(RMVector3 from, RMVector3 to, RMColor color);

        void DrawArc(RMVector3 center, RMVector3 normal, RMVector3 from,
                     float angle, float radius, RMColor color);

        void DrawSphere(RMVector3 center, float radius, RMColor color);

        void DrawWireSphere(RMVector3 center, float radius, RMColor color);
    }
}
