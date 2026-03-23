using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// 运行时可视化实现，使用 Debug.DrawLine。
    /// 适用于运行时环境（Game 视图），持续一帧。
    /// </summary>
    public class UnityRuntimeVisualizer : IEngineVisualizer
    {
        public void DrawLine(RMVector3 from, RMVector3 to, RMColor color)
        {
            Debug.DrawLine(
                TypeConverter.ToUnity(from),
                TypeConverter.ToUnity(to),
                TypeConverter.ToUnity(color));
        }

        public void DrawArc(RMVector3 center, RMVector3 normal, RMVector3 from,
                            float angle, float radius, RMColor color)
        {
            // 用折线段近似弧线
            const int segments = 24;
            var uCenter = TypeConverter.ToUnity(center);
            var uNormal = TypeConverter.ToUnity(normal).normalized;
            var uFrom = TypeConverter.ToUnity(from).normalized * radius;
            var uColor = TypeConverter.ToUnity(color);
            float step = angle / segments;

            var prev = uCenter + uFrom;
            for (int i = 1; i <= segments; i++)
            {
                float a = step * i;
                var dir = Quaternion.AngleAxis(a * Mathf.Rad2Deg, uNormal) * uFrom;
                var curr = uCenter + dir;
                Debug.DrawLine(prev, curr, uColor);
                prev = curr;
            }
        }

        public void DrawSphere(RMVector3 center, float radius, RMColor color)
        {
            // Debug.DrawLine 无法绘制实心球体，退化为三圈线框
            DrawWireSphere(center, radius, color);
        }

        public void DrawWireSphere(RMVector3 center, float radius, RMColor color)
        {
            var uCenter = TypeConverter.ToUnity(center);
            var uColor = TypeConverter.ToUnity(color);
            const int segments = 24;
            float step = 2f * Mathf.PI / segments;

            // XY, XZ, YZ 三个平面各画一个圆
            DrawCircle(uCenter, Vector3.forward, Vector3.right, radius, segments, step, uColor);
            DrawCircle(uCenter, Vector3.up, Vector3.right, radius, segments, step, uColor);
            DrawCircle(uCenter, Vector3.up, Vector3.forward, radius, segments, step, uColor);
        }

        private static void DrawCircle(Vector3 center, Vector3 normal, Vector3 startDir,
                                        float radius, int segments, float step, Color color)
        {
            var prev = center + startDir * radius;
            for (int i = 1; i <= segments; i++)
            {
                float a = step * i;
                var dir = Quaternion.AngleAxis(a * Mathf.Rad2Deg, normal) * startDir * radius;
                var curr = center + dir;
                Debug.DrawLine(prev, curr, color);
                prev = curr;
            }
        }
    }
}
