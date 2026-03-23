#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// 编辑器环境下的被动可视化实现，使用 Gizmos API。
    /// 仅在 OnDrawGizmos / OnDrawGizmosSelected 回调中调用才有效。
    /// </summary>
    public class UnityEditorVisualizer : IEngineVisualizer
    {
        public void DrawLine(RMVector3 from, RMVector3 to, RMColor color)
        {
            Gizmos.color = TypeConverter.ToUnity(color);
            Gizmos.DrawLine(TypeConverter.ToUnity(from), TypeConverter.ToUnity(to));
        }

        public void DrawArc(RMVector3 center, RMVector3 normal, RMVector3 from,
                            float angle, float radius, RMColor color)
        {
            Handles.color = TypeConverter.ToUnity(color);
            Handles.DrawWireArc(
                TypeConverter.ToUnity(center),
                TypeConverter.ToUnity(normal),
                TypeConverter.ToUnity(from),
                angle * RMMathUtils.Rad2Deg,
                radius);
        }

        public void DrawSphere(RMVector3 center, float radius, RMColor color)
        {
            Gizmos.color = TypeConverter.ToUnity(color);
            Gizmos.DrawSphere(TypeConverter.ToUnity(center), radius);
        }

        public void DrawWireSphere(RMVector3 center, float radius, RMColor color)
        {
            Gizmos.color = TypeConverter.ToUnity(color);
            Gizmos.DrawWireSphere(TypeConverter.ToUnity(center), radius);
        }
    }
}
#endif
