using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 碰撞事件桥接组件。
    /// 挂载到需要监听碰撞的 GameObject 上，将 Unity 的 OnTrigger 回调转换为 ICollisionListener 调用。
    /// 使用 Trigger 模式而非 Collision 模式，因为机械臂的 Rigidbody 均为 kinematic，
    /// Unity 不会在两个 kinematic Rigidbody 之间触发 OnCollision 事件。
    /// 由 UnityEnginePhysics.RegisterCollisionListener() 自动管理。
    /// </summary>
    public class UnityCollisionBridge : MonoBehaviour
    {
        private ICollisionListener _listener;
        private IEngineObject _selfObject;

        /// <summary>
        /// 设置碰撞监听器和自身物体引用。
        /// </summary>
        public void Setup(ICollisionListener listener, IEngineObject selfObject)
        {
            _listener = listener;
            _selfObject = selfObject;
        }

        /// <summary>
        /// 清除监听器。
        /// </summary>
        public void ClearListener()
        {
            _listener = null;
        }

        private void OnTriggerEnter(Collider other)
        {
            if (_listener == null) return;
            _listener.OnCollisionEnter(ConvertTrigger(other));
        }

        private void OnTriggerStay(Collider other)
        {
            if (_listener == null) return;
            _listener.OnCollisionStay(ConvertTrigger(other));
        }

        private void OnTriggerExit(Collider other)
        {
            if (_listener == null) return;
            _listener.OnCollisionExit(ConvertTrigger(other));
        }

        private RMCollisionEventData ConvertTrigger(Collider other)
        {
            // Trigger 没有接触点和碰撞速度信息，使用两物体间距离和速度近似
            var selfPos = transform.position;
            var otherPos = other.transform.position;

            // 近似相对速度：用 Rigidbody velocity（kinematic 通常为 0，但记录差值可用于评估）
            var selfRb = GetComponent<Rigidbody>();
            var otherRb = other.attachedRigidbody;
            var selfVel = selfRb != null ? selfRb.velocity : Vector3.zero;
            var otherVel = otherRb != null ? otherRb.velocity : Vector3.zero;
            var relVel = otherVel - selfVel;

            // 近似接触面积：使用两碰撞体 bounds 交叉区域
            var selfBounds = GetComponent<Collider>()?.bounds ?? new Bounds(selfPos, Vector3.zero);
            var otherBounds = other.bounds;
            float contactArea = EstimateOverlapArea(selfBounds, otherBounds);

            // 近似接触法线：从 self 指向 other 的方向
            var direction = (otherPos - selfPos);
            var normal = direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.up;

            // 近似接触点：两中心的中点
            var contactPoint = (selfPos + otherPos) * 0.5f;

            return new RMCollisionEventData
            {
                Self = _selfObject,
                Other = new UnityEngineObject(other.transform),
                ContactPoints = new RMVector3[] { TypeConverter.ToRM(contactPoint) },
                RelativeVelocity = TypeConverter.ToRM(relVel),
                ContactArea = contactArea,
                ContactNormal = TypeConverter.ToRM(normal)
            };
        }

        /// <summary>
        /// 估算两个 Bounds 的重叠面积（取重叠区域最大两轴之积）。
        /// </summary>
        private static float EstimateOverlapArea(Bounds a, Bounds b)
        {
            float overlapX = Mathf.Max(0f, Mathf.Min(a.max.x, b.max.x) - Mathf.Max(a.min.x, b.min.x));
            float overlapY = Mathf.Max(0f, Mathf.Min(a.max.y, b.max.y) - Mathf.Max(a.min.y, b.min.y));
            float overlapZ = Mathf.Max(0f, Mathf.Min(a.max.z, b.max.z) - Mathf.Max(a.min.z, b.min.z));

            // 取最大两个分量之积
            float x = overlapX, y = overlapY, z = overlapZ;
            if (x < y) { float t = x; x = y; y = t; }
            if (y < z) { float t = y; y = z; z = t; }
            if (x < y) { float t = x; x = y; y = t; }

            return x * y;
        }
    }
}
