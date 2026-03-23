using System.Collections.Generic;
using UnityEngine;
using RobotMatrix.Math;
using IEngine;

namespace IEngine.Unity
{
    /// <summary>
    /// Unity 物理引擎适配器，实现 IEnginePhysics。
    /// 封装 Unity 的 Collider、Rigidbody、Transform 层级查询等 API。
    /// </summary>
    public class UnityEnginePhysics : IEnginePhysics
    {
        // ===== 碰撞体管理 =====

        public void AddBoxCollider(IEngineObject target, RMVector3 center, RMVector3 size)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var box = go.AddComponent<BoxCollider>();
            box.center = TypeConverter.ToUnity(center);
            box.size = TypeConverter.ToUnity(size);
            box.isTrigger = true;
        }

        public void AddMeshCollider(IEngineObject target, bool convex)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var meshFilter = go.GetComponent<MeshFilter>();
            if (meshFilter == null || meshFilter.sharedMesh == null)
                return;

            var mc = go.AddComponent<MeshCollider>();
            mc.sharedMesh = meshFilter.sharedMesh;
            mc.convex = convex;
            mc.isTrigger = true;
        }

        public RMColliderInfo[] GetColliders(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return new RMColliderInfo[0];

            var colliders = go.GetComponents<Collider>();
            var infos = new RMColliderInfo[colliders.Length];

            for (int i = 0; i < colliders.Length; i++)
            {
                infos[i] = new RMColliderInfo
                {
                    Type = GetColliderType(colliders[i]),
                    Enabled = colliders[i].enabled,
                    OwnerName = go.name
                };
            }

            return infos;
        }

        public void SetCollidersEnabled(IEngineObject target, bool enabled)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var colliders = go.GetComponents<Collider>();
            for (int i = 0; i < colliders.Length; i++)
                colliders[i].enabled = enabled;
        }

        public void SetCollidersIsTrigger(IEngineObject target, bool isTrigger)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var colliders = go.GetComponents<Collider>();
            for (int i = 0; i < colliders.Length; i++)
                colliders[i].isTrigger = isTrigger;
        }

        public void RemoveAllColliders(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var colliders = go.GetComponents<Collider>();
            for (int i = colliders.Length - 1; i >= 0; i--)
                Object.Destroy(colliders[i]);
        }

        // ===== 层级查询 =====

        public IEngineObject[] GetAllDescendants(IEngineObject root)
        {
            var transform = GetTransform(root);
            if (transform == null) return new IEngineObject[0];

            var all = transform.GetComponentsInChildren<Transform>(true);
            var result = new IEngineObject[all.Length];
            for (int i = 0; i < all.Length; i++)
                result[i] = new UnityEngineObject(all[i]);

            return result;
        }

        public IEngineObject[] GetDescendantsUntil(IEngineObject root, IEngineObject stopBefore)
        {
            var rootTransform = GetTransform(root);
            var stopTransform = GetTransform(stopBefore);
            if (rootTransform == null) return new IEngineObject[0];

            var result = new List<IEngineObject>();
            CollectDescendantsUntil(rootTransform, stopTransform, result);
            return result.ToArray();
        }

        private void CollectDescendantsUntil(Transform current, Transform stopBefore, List<IEngineObject> result)
        {
            result.Add(new UnityEngineObject(current));

            for (int i = 0; i < current.childCount; i++)
            {
                var child = current.GetChild(i);
                // 遇到 stopBefore 节点时跳过该子树
                if (stopBefore != null && child == stopBefore)
                    continue;
                CollectDescendantsUntil(child, stopBefore, result);
            }
        }

        // ===== 碰撞事件 =====

        public void RegisterCollisionListener(IEngineObject target, ICollisionListener listener)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var bridge = go.GetComponent<UnityCollisionBridge>();
            if (bridge == null)
                bridge = go.AddComponent<UnityCollisionBridge>();

            bridge.Setup(listener, target);
        }

        public void UnregisterCollisionListener(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var bridge = go.GetComponent<UnityCollisionBridge>();
            if (bridge != null)
                bridge.ClearListener();
        }

        // ===== Rigidbody 管理 =====

        public void EnsureKinematicRigidbody(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return;

            var rb = go.GetComponent<Rigidbody>();
            if (rb == null)
            {
                rb = go.AddComponent<Rigidbody>();
                rb.isKinematic = true;
                rb.useGravity = false;
            }
            else if (!rb.isKinematic)
            {
                rb.isKinematic = true;
                rb.useGravity = false;
            }
        }

        // ===== 碰撞体边界查询 =====

        public RMBounds GetColliderBounds(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return RMBounds.Invalid;

            var colliders = go.GetComponents<Collider>();
            if (colliders.Length == 0) return RMBounds.Invalid;

            // 合并所有碰撞体的包围盒
            var bounds = colliders[0].bounds;
            for (int i = 1; i < colliders.Length; i++)
                bounds.Encapsulate(colliders[i].bounds);

            return new RMBounds(
                TypeConverter.ToRM(bounds.center),
                TypeConverter.ToRM(bounds.size));
        }

        // ===== 碰撞体形状查询 =====

        public RMColliderShape[] GetColliderShapes(IEngineObject target)
        {
            var go = GetGameObject(target);
            if (go == null) return new RMColliderShape[0];

            var colliders = go.GetComponents<Collider>();
            if (colliders.Length == 0) return new RMColliderShape[0];

            var shapes = new List<RMColliderShape>();
            for (int i = 0; i < colliders.Length; i++)
            {
                if (!colliders[i].enabled) continue;
                var shape = BuildColliderShape(colliders[i]);
                shapes.Add(shape);
            }
            return shapes.ToArray();
        }

        private static RMColliderShape BuildColliderShape(Collider collider)
        {
            var shape = new RMColliderShape();
            var t = collider.transform;
            shape.WorldRotation = TypeConverter.ToRM(t.rotation);

            if (collider is BoxCollider box)
            {
                shape.Type = RMColliderType.Box;
                shape.WorldCenter = TypeConverter.ToRM(t.TransformPoint(box.center));
                var lossyScale = t.lossyScale;
                shape.BoxSize = new RMVector3(
                    box.size.x * lossyScale.x,
                    box.size.y * lossyScale.y,
                    box.size.z * lossyScale.z);
            }
            else if (collider is SphereCollider sphere)
            {
                shape.Type = RMColliderType.Sphere;
                shape.WorldCenter = TypeConverter.ToRM(t.TransformPoint(sphere.center));
                var lossyScale = t.lossyScale;
                float maxScale = Mathf.Max(Mathf.Abs(lossyScale.x),
                    Mathf.Max(Mathf.Abs(lossyScale.y), Mathf.Abs(lossyScale.z)));
                shape.SphereRadius = sphere.radius * maxScale;
            }
            else if (collider is CapsuleCollider capsule)
            {
                shape.Type = RMColliderType.Capsule;
                shape.WorldCenter = TypeConverter.ToRM(t.TransformPoint(capsule.center));
                var lossyScale = t.lossyScale;
                int dir = capsule.direction; // 0=X, 1=Y, 2=Z
                shape.CapsuleDirection = dir;

                float radiusScaleA, radiusScaleB, heightScale;
                if (dir == 0)      { radiusScaleA = Mathf.Abs(lossyScale.y); radiusScaleB = Mathf.Abs(lossyScale.z); heightScale = Mathf.Abs(lossyScale.x); }
                else if (dir == 1) { radiusScaleA = Mathf.Abs(lossyScale.x); radiusScaleB = Mathf.Abs(lossyScale.z); heightScale = Mathf.Abs(lossyScale.y); }
                else               { radiusScaleA = Mathf.Abs(lossyScale.x); radiusScaleB = Mathf.Abs(lossyScale.y); heightScale = Mathf.Abs(lossyScale.z); }

                shape.CapsuleRadius = capsule.radius * Mathf.Max(radiusScaleA, radiusScaleB);
                shape.CapsuleHeight = capsule.height * heightScale;
            }
            else if (collider is MeshCollider mc)
            {
                shape.Type = RMColliderType.Mesh;
                shape.WorldCenter = TypeConverter.ToRM(collider.bounds.center);

                var mesh = mc.sharedMesh;
                if (mesh != null)
                {
                    var localVerts = mesh.vertices;
                    var worldVerts = new RMVector3[localVerts.Length];
                    for (int v = 0; v < localVerts.Length; v++)
                        worldVerts[v] = TypeConverter.ToRM(t.TransformPoint(localVerts[v]));
                    shape.MeshVertices = worldVerts;
                    shape.MeshTriangles = mesh.triangles;
                }
            }

            return shape;
        }

        // ===== 内部工具方法 =====

        private static GameObject GetGameObject(IEngineObject obj)
        {
            if (obj is UnityEngineObject ueo && ueo.UnityTransform != null)
                return ueo.UnityTransform.gameObject;
            return null;
        }

        private static Transform GetTransform(IEngineObject obj)
        {
            if (obj is UnityEngineObject ueo)
                return ueo.UnityTransform;
            return null;
        }

        private static RMColliderType GetColliderType(Collider collider)
        {
            if (collider is BoxCollider) return RMColliderType.Box;
            if (collider is MeshCollider) return RMColliderType.Mesh;
            if (collider is SphereCollider) return RMColliderType.Sphere;
            if (collider is CapsuleCollider) return RMColliderType.Capsule;
            return RMColliderType.Box;
        }
    }
}
